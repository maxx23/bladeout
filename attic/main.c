#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <libbladeRF.h>


#define DEFAULT_FREQUENCY	300000000
#define DEFAULT_SAMPLERATE	1000000
#define DEFAULT_BANDWIDTH	(DEFAULT_SAMPLERATE * 3 / 4)
#define DEFAULT_TXVGA1		-35
#define DEFAULT_TXVGA2		0
#define DEFAULT_BUFFERS		16
#define DEFAULT_SAMPLES		16384
#define DEFAULT_TRANSFERS	(DEFAULT_BUFFERS / 2)
#define DEFAULT_GAIN		1.f
#define DEFAULT_AGAIN		0.f
#define DEFAULT_DEVICE_ID	""


/* Buffer structure with attached configuration
 */
struct buffer_s
{
	float *fbuf;
	void **sbuf;
	float gain;
	float again;
	unsigned int pos;
	unsigned int num_buffers;
	unsigned int num_samples;
	unsigned int num_transfers;
};

/* All the device parameters and the device itself
 */
struct devinfo_s
{
	char *device_id;
	struct bladerf *dev;
	struct bladerf_stream *stream;
	unsigned int samplerate;
	unsigned int bandwidth;
	unsigned int frequency;
	int txvga1;
	int txvga2;
	struct buffer_s buffers;
};

/* Just this one state */
static int running = true;

/* Display usage information
 */
void usage(char *name, struct devinfo_s *dev)
{
	fprintf(stderr, "%s <options>\n"
		"\t-h\t\tShow this help text.\n"
		"\t-d <device_id>\tDevice string (default: \"%s\").\n"
		"\t-f <frequency>\tFrequency (default: %iHz).\n"
		"\t-r <rate>\tSamplerate (default: %i).\n"
		"\t-b <bandwidth>\tLPF bandwidth (current: %iHz).\n"
		"\t-g <txvga1>\tGain for txvga1 (default: %idB).\n"
		"\t-G <txvga2>\tGain for txvga2 (default: %idB).\n"
		"\t-m <gain>\tSoft gain (default: %f).\n"
		"\t-a <autogain>\tAuto gain adjustment (current: %f).\n"
		"\t-n <buffers>\tNumber of buffers (default: %i).\n"
		"\t-s <samples>\tSamples per buffer (default: %i).\n"
		"\t-t <transfers>\tMaximum concurrent transfers (current: %i).\n"
		"\n",
		name,
		dev->device_id,
		dev->frequency,
		dev->samplerate,
		dev->bandwidth?dev->bandwidth:DEFAULT_BANDWIDTH,
		dev->txvga1,
		dev->txvga2,
		dev->buffers.gain,
		dev->buffers.again,
		dev->buffers.num_buffers,
		dev->buffers.num_samples,
		dev->buffers.num_transfers?dev->buffers.num_transfers:DEFAULT_TRANSFERS
	);
}

/* Signal handler
 */
static void sighandler(int signum)
{
	switch(signum) {
		default:
			fprintf(stderr, "Signal %d caught, exiting.\n", signum);
			running = false;
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
	int16_t *ptr;
	struct buffer_s *buf = (struct buffer_s *)(user_data);
	size_t nread;
	int n;

	/* Check for the one state */
	if(!running)
	{
		ptr = NULL;
		goto out;
	}
	
	/* Read the (float) samples */
	nread = fread(buf->fbuf, num_samples * sizeof(float), 2, stdin);
	if(nread < 2)
		fprintf(stderr, "WARNING: Short read.\n");
	
	/* Check a few conditions */
	if(feof(stdin) || ferror(stdin))
	{
		running = false;
		ptr = NULL;
		goto out;
	}
	
	/* Get the current slot in the buffers */
	ptr = (int16_t *)buf->sbuf[buf->pos];
	
	/* Convert float -> int16 and auto gain control */
	for(n = 0; n < num_samples; n++)
	{
		float i_in = buf->fbuf[n * 2];
		float q_in = buf->fbuf[n * 2 + 1];
		
		/* Multiply by current soft gain */
		float i = i_in * buf->gain;
		float q = q_in * buf->gain;
		
		/* Calculate magnitude */
		float s = sqrtf(i * i + q * q);
		
		/* Check if auto gain control is enabled and
		 * magnitude is over bounds */
		if((buf->again > 0.0) && (s >= buf->again))
		{
			/* Scale down current gain accordingly */
			buf->gain = buf->again/s * buf->gain;
			
			fprintf(stderr, "WARNING: Soft gain adjusted to %f (%f).\n",
				buf->gain, s);
			
			/* Also correct current samples using the new gain */
			i = i_in * buf->gain;
			q = q_in * buf->gain;
		}

		/* Convert to int16 and write to output buffer */
		ptr[n * 2] = (int16_t)(i * 2047.f);
		ptr[n * 2 + 1] = (int16_t)(q * 2047.f);
	}
	
	/* Advance to the next slot */
	buf->pos = (buf->pos + 1) % buf->num_buffers;

out:
	return ptr;
}

/* Initialization and stuff
 */
int main(int argc, char **argv)
{
	struct bladerf_devinfo *devs;
	struct sigaction sigact;
	struct devinfo_s device;
	int show_help = false;
	int n, ret;
	char ch;

	/* Set up default values, bandwidth and num_transfers
	 * are automatically calculated later */
	device.device_id = DEFAULT_DEVICE_ID;
	device.frequency = DEFAULT_FREQUENCY;
	device.samplerate = DEFAULT_SAMPLERATE;
	device.bandwidth = 0;
	device.txvga1 = DEFAULT_TXVGA1;
	device.txvga2 = DEFAULT_TXVGA2;

	device.buffers.gain = DEFAULT_GAIN;
	device.buffers.again = DEFAULT_AGAIN;
	device.buffers.num_buffers = DEFAULT_BUFFERS;
	device.buffers.num_samples = DEFAULT_SAMPLES;
	device.buffers.num_transfers = 0;
	device.buffers.pos = 0;

	/* Evaluate command line options */
	while((ch = getopt(argc, argv, "hd:f:r:b:g:G:a:m:n:s:t:")) != -1)
	{
		switch(ch)
		{
			case 'd':
				device.device_id = optarg; break;
			case 'f':
				device.frequency = atoi(optarg); break;
			case 'r':
				device.samplerate = atoi(optarg); break;
			case 'b':
				device.bandwidth = atoi(optarg); break;
			case 'g':
				device.txvga1 = atoi(optarg); break;
			case 'G':
				device.txvga2 = atoi(optarg); break;
			case 'm':
				device.buffers.gain = atof(optarg); break;
			case 'a':
				device.buffers.again = atof(optarg); break;
			case 'n':
				device.buffers.num_buffers = atoi(optarg); break;
			case 's':
				device.buffers.num_samples = atoi(optarg); break;
			case 't':
				device.buffers.num_transfers = atoi(optarg); break;
			case 'h':
			default:
				show_help = true;
		}
	}

	/* Now calculate bandwidth and num_transfers if the user didn't
	 * configure them manually */
	if(device.bandwidth == 0)
		device.bandwidth = device.samplerate * 3 / 4;
	if(device.buffers.num_transfers == 0)
		device.buffers.num_transfers = device.buffers.num_buffers / 2;

	if(show_help)
	{
		usage(argv[0], &device);
		return EXIT_FAILURE;
	}
	
	argc -= optind;
	argv += optind;

	/* Allocate the float input buffer */
	device.buffers.fbuf =
		malloc(device.buffers.num_samples * 2 * sizeof(float));
	
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
			"Instance:\t%i\n\n",
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

	/* Set the device parameters */
	ret = bladerf_set_sample_rate(device.dev,
		BLADERF_MODULE_TX, device.samplerate, &device.samplerate);
	if(ret != 0)
	{
		fprintf(stderr, "Error setting sample rate to %i: %s.\n",
			device.samplerate, bladerf_strerror(ret));
		goto out1;
	}
	else
	{
		fprintf(stderr, "Actual sample rate is %i.\n",
			device.samplerate);
	}

	ret = bladerf_set_frequency(device.dev,
		BLADERF_MODULE_TX, device.frequency);
	if(ret != 0)
	{
		fprintf(stderr, "Error setting frequency to %iHz: %s.\n",
			device.frequency, bladerf_strerror(ret));
		goto out1;
	}
	else
	{
		fprintf(stderr, "Frequency set to %iHz.\n", device.frequency);
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
		fprintf(stderr, "Bandwidth set to %iHz.\n", device.bandwidth);
	}

	/* Set up the sample stream */
	ret = bladerf_init_stream(&device.stream,
		device.dev, stream_callback, &device.buffers.sbuf,
		device.buffers.num_buffers,	BLADERF_FORMAT_SC16_Q12,
		device.buffers.num_samples, device.buffers.num_transfers,
		&device.buffers);
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
	return EXIT_SUCCESS;
}

