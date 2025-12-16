#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <drivers/drv_hrt.h>
#include <cstdlib>

extern "C" __EXPORT int regpeek_main(int argc, char *argv[]);

static void usage()
{
	PX4_INFO("Usage: regpeek [-a <addr>] [-w <value>]");
	PX4_INFO("  -a <addr>   Register address (default: 0x400AC004)");
	PX4_INFO("  -w <value>  Value to write to the register");
}

int regpeek_main(int argc, char *argv[])
{
	uint32_t addr = 0x400AC004; // default address
	uint32_t value_to_write = 0;
	bool write_mode = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "a:w:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			addr = strtoul(myoptarg, nullptr, 0);
			break;
		case 'w':
			value_to_write = strtoul(myoptarg, nullptr, 0);
			write_mode = true;
			break;
		default:
			usage();
			return -1;
		}
	}

	volatile uint32_t *reg = (uint32_t *)addr;

	if (write_mode) {
		PX4_INFO("Writing 0x%08X to register 0x%08X", (unsigned int)value_to_write, (unsigned int)addr);
		*reg = value_to_write;
		px4_usleep(10); // pequena espera para garantir que a escrita seja completada
	}

	uint32_t read_back = *reg;
	PX4_INFO("Read back from 0x%08X: 0x%08X", (unsigned int)addr, (unsigned int)read_back);

	return 0;
}

