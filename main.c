#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <libbladeRF.h>


/* Default values */
#define DEFAULT_FREQUENCY	300000000
#define DEFAULT_SAMPLERATE	1000000
#define DEFAULT_BANDWIDTH	(DEFAULT_SAMPLERATE * 3 / 4)
#define DEFAULT_TXVGA1		-35
#define DEFAULT_TXVGA2		0
#define DEFAULT_BUFFERS		16
#define DEFAULT_CB_SIZE		256
#define DEFAULT_SAMPLES		16384
#define DEFAULT_TRANSFERS	(DEFAULT_BUFFERS / 2)
#define DEFAULT_GAIN		1.f
#define DEFAULT_AGAIN		0.f
#define DEFAULT_DEVICE_ID	""

#define DEFAULT_READ_BLOCKSIZE	4096

/* The conversion loop can be unrolled automatically by the compiler */
#define UNROLL_FACTOR		8

/* States */
#define STATE_RUNNING		0
#define STATE_EXIT			1
#define STATE_FINISHED		2

/* Management structure for circular input buffer
 */
struct cb_s
{
	unsigned int w;					/* Write position */
	unsigned int r;					/* Read position */
	int16_t *data;					/* Actual buffer */
	float *fbuf;					/* float buffer for conversion */
	unsigned int size;				/* Number of elements */
	unsigned int r_size;			/* Blocksize for fread() */
	pthread_mutex_t r_lock;			/* For locking changes in r pointer */
	pthread_mutex_t w_lock;			/* For locking changes in w pointer */
	pthread_cond_t e_cond;			/* Empty condition */
	pthread_cond_t f_cond;			/* Full condition */
	pthread_mutex_t e_cond_lock;
	pthread_mutex_t f_cond_lock;
};

/* Buffer management structure
 */
struct buffer_s
{
	void **sbuf;				/* Device buffers */
	struct cb_s cb;				/* Circular buffers */
	float gain;					/* Current soft gain */
	float again;				/* Auto gain setting */
	unsigned int pos;			/* Position in device buffers */
	unsigned int num_buffers;	/* # slots */
	unsigned int num_samples;	/* Samples per slot */
	unsigned int num_transfers;	/* Maximum concurrent transfers */
};

/* All the device parameters and the device itself
 */
struct devinfo_s
{
	char *device_id;				/* Device identifier */
	struct bladerf *dev;			/* Device info */
	struct bladerf_stream *stream;	/* Stream info */
	unsigned int samplerate;		/* ... */
	unsigned int bandwidth;
	unsigned int frequency;
	int txvga1;						/* TXVGA1 gain in dB */
	int txvga2;						/* TXVGA2 gain in dB */
	struct buffer_s buffers;		/* Buffer management */
};

/* Just this one state */
static int state = STATE_RUNNING;

/* Display usage information
 */
static void usage(char *name, struct devinfo_s *dev)
{
	fprintf(stderr, "%s <options>\n"
		"\t-h\t\tShow this help text.\n"
		"\t-d <device_id>\tDevice string (current: \"%s\").\n"
		"\t-f <frequency>\tFrequency (current: %uHz).\n"
		"\t-r <rate>\tSamplerate (current: %u).\n"
		"\t-b <bandwidth>\tLPF bandwidth (current: %uHz).\n"
		"\t-g <txvga1>\tGain for txvga1 (current: %idB).\n"
		"\t-G <txvga2>\tGain for txvga2 (current: %idB).\n"
		"\t-m <gain>\tSoft gain (current: %f).\n"
		"\t-a <autogain>\tAuto gain adjustment (current: %f).\n"
		"\t-p <prebuffer>\tCircular buffer size (current: %u).\n"
		"\t-n <buffers>\tNumber of device buffers (current: %u).\n"
		"\t-s <samples>\tSamples per buffer (current: %u).\n"
		"\t-t <transfers>\tMaximum concurrent transfers (current: %u).\n"
		"\t-R <blocksize>\tBlocksize for read operations (current: %u).\n"
		"\n",
		name,
		dev->device_id,
		dev->frequency,
		dev->samplerate,
		dev->bandwidth,
		dev->txvga1,
		dev->txvga2,
		dev->buffers.gain,
		dev->buffers.again,
		dev->buffers.cb.size,
		dev->buffers.num_buffers,
		dev->buffers.num_samples,
		dev->buffers.num_transfers,
		dev->buffers.cb.r_size
	);

	fprintf(stderr, "Circular buffer size: %lukB.\n"
		"Device buffer size: %lukB.\n"
		"Float buffer size: %lukB.\n",
		(dev->buffers.cb.size * dev->buffers.num_samples * 2
			* sizeof(int16_t)) >> 10,
		(dev->buffers.num_buffers * dev->buffers.num_samples * 2
			* sizeof(int16_t)) >> 10,
		(dev->buffers.num_samples * sizeof(float) * 2) >> 10
	);
}

/* Signal handler
 */
static void sighandler(int signum)
{
	switch(signum) {
		default:
			fprintf(stderr, "Signal %d caught, exiting.\n", signum);
			state |= STATE_EXIT;
	}
}

/* This gets called when the bladeRF needs more data
 */
static void *stream_callback(
		struct bladerf *device,
		struct bladerf_stream *stream,
		struct bladerf_metadata *metadata,
		void *samples,
		size_t num_samples,
		void *user_data)
{
	int16_t *rptr, *wptr;
	struct buffer_s *buf = (struct buffer_s *)(user_data);
	struct cb_s *cb = &buf->cb;
	unsigned int tmp_w;

	/* User wants to stop NOW */
	if(state & STATE_EXIT)
	{
		wptr = NULL;
		goto out;
	}
	
	/* Get write pointer for comparison in a safe way */
	pthread_mutex_lock(&cb->w_lock);
	tmp_w = cb->w;
	pthread_mutex_unlock(&cb->w_lock);

	/* Check if empty */
	if(tmp_w == cb->r)
	{
		printf("WARNING: Input buffer underrun.\n");
		
		/* If input buffer is empty (EOF or something) just exit */
		if(state & STATE_FINISHED)
		{
			wptr = NULL;
			goto out;
		}
	
		/* If the input still goes on we need to wait */
		pthread_cond_wait(&cb->e_cond, &cb->e_cond_lock);
	}
	
	/* Get current position in input ring buffer */
	rptr = &cb->data[buf->num_samples * 2 * (cb->r & (cb->size - 1))];
	
	/* Get the current slot in the target buffers */
	wptr = (int16_t *)buf->sbuf[buf->pos];
	
	/* I'd like to avoid, but... */
	memcpy(wptr, rptr, buf->num_samples * 2 * sizeof(int16_t));
	
	/* Advance to the next slot */
	buf->pos = (buf->pos + 1) % buf->num_buffers;

	/* Advance input ring pointer */
	pthread_mutex_lock(&cb->r_lock);
	cb->r = (cb->r + 1) & (2 * cb->size - 1);
	pthread_mutex_unlock(&cb->r_lock);

	/* Signal that buffer is now filled again */
	pthread_cond_signal(&cb->f_cond);
	
	
out:
	return wptr;
}


/* Read, convert and scale input data
 */
static void *reader_proc(void *arg)
{
	struct buffer_s *buf = (struct buffer_s *)(arg);
	struct cb_s *cb = &buf->cb;
	unsigned int tmp_r;
	int16_t *ptr;
	size_t nread;
	unsigned int n, m;
	const unsigned int n_blocks = sizeof(float) * 2
		* buf->num_samples / cb->r_size;

	while(!state)
	{
		/* Get read pointer in a safe way */
		pthread_mutex_lock(&cb->r_lock);
		tmp_r = cb->r;
		pthread_mutex_unlock(&cb->r_lock);

		/* Check for overflow (full condition)
		 * Wait until consumer signals a free slot */
		if(cb->w == (tmp_r ^ cb->size))
			pthread_cond_wait(&cb->f_cond, &cb->f_cond_lock);

		/* User wants to exit now */
		if(state & STATE_EXIT)
			break;

		/* Read the (float) samples */
		nread = fread(cb->fbuf, cb->r_size,
			n_blocks, stdin);

		if(nread < n_blocks)
			fprintf(stderr, "WARNING: Short read.\n");
		
		/* Check a few conditions */
		if(feof(stdin) || ferror(stdin))
		{
			state |= STATE_FINISHED;
			break;
		}
		
		/* Get the current slot in the buffers */
		ptr = &cb->data[buf->num_samples * 2 * (cb->w & (cb->size - 1))];
	
		/* Convert float -> int16 and auto gain control */
		for(n = 0; n < buf->num_samples; n += UNROLL_FACTOR)
		{
			/* Allow for unrolling */
			for(m = 0; m < UNROLL_FACTOR; m++)
			{
				float i_in = cb->fbuf[(n + m) * 2];
				float q_in = cb->fbuf[(n + m) * 2 + 1];
				
				/* Multiply by current soft gain */
				float i = i_in * buf->gain;
				float q = q_in * buf->gain;
				
				/* Calculate magnitude */
				float s = sqrtf(i * i + q * q);
				
				/* Check if auto gain control is enabled and
				 * magnitude is over bounds */
				if((buf->again > 0.0) && (s > buf->again))
				{
					/* Scale down current gain accordingly */
					buf->gain = buf->again/s * buf->gain;
					
					fprintf(stderr,
						"WARNING: Soft gain adjusted to %f (%f).\n",
						buf->gain, s);
					
					/* Also correct current samples using the new gain */
					i = i_in * buf->gain;
					q = q_in * buf->gain;
				}

				/* Convert to int16 and write to output buffer */
				ptr[(n + m) * 2] = (int16_t)(i * 2047.f);
				ptr[(n + m) * 2 + 1] = (int16_t)(q * 2047.f);
			}
		}
		
		/* Advance write pointer in a safe way */
		pthread_mutex_lock(&cb->w_lock);
		cb->w = (cb->w + 1) & (2 * cb->size - 1);
		pthread_mutex_unlock(&cb->w_lock);

		/* Signal a full slot */
		pthread_cond_signal(&cb->e_cond);
	}

	pthread_exit(NULL);
}

/* Initialization and stuff
 */
int main(int argc, char **argv)
{
	struct bladerf_devinfo *devs;
	struct sigaction sigact;
	struct devinfo_s device;
	bool show_help = false;
	int n, ret;
	int ch;
	struct cb_s *cb;
	struct buffer_s *buf;
	pthread_t reader;


	buf = &device.buffers;
	cb = &buf->cb;

	/* Set up default values, bandwidth and num_transfers
	 * are automatically calculated later */
	device.device_id = DEFAULT_DEVICE_ID;
	device.frequency = DEFAULT_FREQUENCY;
	device.samplerate = DEFAULT_SAMPLERATE;
	device.bandwidth = 0;
	device.txvga1 = DEFAULT_TXVGA1;
	device.txvga2 = DEFAULT_TXVGA2;

	buf->pos = 0;
	buf->gain = DEFAULT_GAIN;
	buf->again = DEFAULT_AGAIN;
	buf->num_buffers = DEFAULT_BUFFERS;
	buf->num_samples = DEFAULT_SAMPLES;
	buf->num_transfers = 0;

	cb->size = DEFAULT_CB_SIZE;
	cb->r_size = DEFAULT_READ_BLOCKSIZE;
	cb->r = 0;
	cb->w = 0;
	pthread_mutex_init(&cb->r_lock, NULL);
	pthread_mutex_init(&cb->w_lock, NULL);
	pthread_mutex_init(&cb->e_cond_lock, NULL);
	pthread_mutex_init(&cb->f_cond_lock, NULL);
	pthread_cond_init(&cb->e_cond, NULL);
	pthread_cond_init(&cb->f_cond, NULL);

	/* Evaluate command line options */
	while((ch = getopt(argc, argv, "hd:f:r:b:g:G:a:m:n:p:s:t:R:")) != -1)
	{
		switch(ch)
		{
			case 'd': device.device_id = optarg; break;
			case 'f': device.frequency = (unsigned int)atoi(optarg); break;
			case 'r': device.samplerate = (unsigned int)atoi(optarg); break;
			case 'b': device.bandwidth = (unsigned int)atoi(optarg); break;
			case 'g': device.txvga1 = atoi(optarg); break;
			case 'G': device.txvga2 = atoi(optarg); break;
			case 'm': buf->gain = (float)atof(optarg); break;
			case 'a': buf->again = (float)atof(optarg); break;
			case 'p': cb->size = (unsigned int)atoi(optarg); break;
			case 'n': buf->num_buffers = (unsigned int)atoi(optarg); break;
			case 's': buf->num_samples = (unsigned int)atoi(optarg); break;
			case 't': buf->num_transfers = (unsigned int)atoi(optarg); break;
			case 'R': cb->r_size = (unsigned int)atoi(optarg); break;
			case 'h':
			default:
				show_help = true;
		}
	}

	/* Now calculate bandwidth and num_transfers if the user didn't
	 * configure them manually */
	if(!device.bandwidth)
		device.bandwidth = device.samplerate * 3 / 4;
	if(!buf->num_transfers)
		buf->num_transfers = buf->num_buffers / 2;

	if(show_help)
	{
		usage(argv[0], &device);
		return EXIT_FAILURE;
	}
	
	argc -= optind;
	argv += optind;

	/* Allocate the buffers */
	cb->data = malloc(cb->size * buf->num_samples * 2 * sizeof(int16_t));
	cb->fbuf = malloc(buf->num_samples * 2 * sizeof(float));


	/* Set up signal handler to enable clean shutdowns */
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);

	/* Look for devices attached */
	ret = bladerf_get_device_list(&devs);
	if(ret < 1)
	{
		fprintf(stderr, "No devices found.\n");
		return EXIT_FAILURE;
	}

	/* Print some information about all the devices */
	for(n = 0; n < ret; n++)
	{
		fprintf(stderr, 
			"Serial:\t%s\n"
			"USB bus:\t%i\n"
			"USB address:\t%i\n"
			"Instance:\t%u\n\n",
			devs[n].serial,
			devs[n].usb_bus,
			devs[n].usb_addr,
			devs[n].instance
		);
	}

	/* the list is not needed any more */
	bladerf_free_device_list(devs);


	/* Open a device by given device string
	 */
	ret = bladerf_open(&device.dev, device.device_id);
	if(ret != 0)
	{
		fprintf(stderr, "Error opening device %s: %s.\n",
			device.device_id, bladerf_strerror(ret));
		goto out0;
	}
	else
	{
		fprintf(stderr, "Device \"%s\" opened successfully.\n",
			device.device_id);
	}

	
	/* Fire up reader thread */
	ret = pthread_create(&reader, NULL, reader_proc,
		(void *)(buf));
	if(ret)
	{
		fprintf(stderr, "Error creating reader thread.\n");
		goto out1;
	}
	else
	{
		fprintf(stderr, "Reader thread fired up.\n");
	}

	fprintf(stderr, "Waiting for buffer to fill up.\n");

	while(!state && (cb->w != (cb->r ^ cb->size)))
		usleep(100000);

	if(state & (STATE_EXIT | STATE_FINISHED))
		goto out1;


	/* Set the device parameters */
	ret = bladerf_set_sample_rate(device.dev,
		BLADERF_MODULE_TX, device.samplerate, &device.samplerate);
	if(ret != 0)
	{
		fprintf(stderr, "Error setting sample rate to %u: %s.\n",
			device.samplerate, bladerf_strerror(ret));
		goto out1;
	}
	else
	{
		fprintf(stderr, "Actual sample rate is %u.\n",
			device.samplerate);
	}

	ret = bladerf_set_frequency(device.dev,
		BLADERF_MODULE_TX, device.frequency);
	if(ret != 0)
	{
		fprintf(stderr, "Error setting frequency to %uHz: %s.\n",
			device.frequency, bladerf_strerror(ret));
		goto out1;
	}
	else
	{
		fprintf(stderr, "Frequency set to %uHz.\n", device.frequency);
	}

	ret = bladerf_set_txvga1(device.dev, device.txvga1);
	if(ret != 0)
	{
		fprintf(stderr, "Error setting gain for txvga1: %s.\n",
			bladerf_strerror(ret));
		goto out1;
	}

	ret = bladerf_set_txvga2(device.dev, device.txvga2);
	if(ret != 0)
	{
		fprintf(stderr, "Error setting gain for txvga2: %s.\n",
			bladerf_strerror(ret));
		goto out1;
	}

	ret = bladerf_set_bandwidth(device.dev,
		BLADERF_MODULE_TX, device.bandwidth, &device.bandwidth);
	if(ret != 0)
	{
		fprintf(stderr, "Error setting LPF bandwidth: %s.\n",
			bladerf_strerror(ret));
		goto out1;
	}
	else
	{
		fprintf(stderr, "Bandwidth set to %uHz.\n", device.bandwidth);
	}

	/* Set up the sample stream */
	ret = bladerf_init_stream(&device.stream,
		device.dev, stream_callback, &buf->sbuf,
		buf->num_buffers,	BLADERF_FORMAT_SC16_Q12,
		buf->num_samples, buf->num_transfers,
		buf);
	if(ret != 0)
	{
		fprintf(stderr, "Failed setting up stream: %s.\n",
			bladerf_strerror(ret));
		goto out1;
	}
	

	/* Finally enable TX... */
	ret = bladerf_enable_module(device.dev, BLADERF_MODULE_TX, true);
	if(ret != 0)
	{
		fprintf(stderr, "Error enabling TX module: %s.\n",
			bladerf_strerror(ret));
		goto out1;
	}
	else
	{
		fprintf(stderr, "Successfully enabled TX module.\n");
	}

	/* ...and start the stream.
	 * Execution stops here until stream has finished. */
	ret = bladerf_stream(device.stream, BLADERF_MODULE_TX);
	if(ret != 0)
	{
		fprintf(stderr, "Failed starting stream: %s.\n",
			bladerf_strerror(ret));
		goto out2;
	}

	/* Cleanup the mess */
out2:
	bladerf_deinit_stream(device.stream);

out1:
	ret = bladerf_enable_module(device.dev, BLADERF_MODULE_TX, false);
	if(ret != 0)
	{
		fprintf(stderr, "Error disabling TX module: %s.\n",
			bladerf_strerror(ret));
	}
	else
	{
		fprintf(stderr, "Successfully disabled TX module.\n");
	}
	
	bladerf_close(device.dev);
	fprintf(stderr, "Device closed.\n");
	
out0:
	pthread_cond_signal(&cb->f_cond);
	pthread_cond_signal(&cb->e_cond);

	pthread_join(reader, NULL);

	free(cb->data);
	free(cb->fbuf);

	return EXIT_SUCCESS;
}

