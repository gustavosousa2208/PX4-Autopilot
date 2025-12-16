#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_config.h>

#include <cstdlib>
#include <cstring>
#include "../../../platforms/nuttx/NuttX/nuttx/arch/arm/src/imxrt/hardware/imxrt_enet.h"

#define I2C_BUS   3
#define I2C_ADDR  0x48
#define I2C_FREQ  100000

extern "C" __EXPORT int sepeek_main(int argc, char *argv[]);

int sepeek_main(int argc, char *argv[])
{
	PX4_INFO("Verificando ACK do SE051 no endereco 0x%02X no I2C bus %d...", I2C_ADDR, I2C_BUS);

	struct i2c_master_s *i2c = px4_i2cbus_initialize(I2C_BUS);

	if (!i2c) {
		PX4_ERR("Erro ao inicializar barramento I2C %d", I2C_BUS);
		return -1;
	}

	uint8_t dummy = 0x00;
	int ret = -1;

	for (int i = 0; i < 10; i++) {
		usleep(1000);
		struct i2c_msg_s msg = {
			.frequency = I2C_FREQ,
			.addr = I2C_ADDR,
			.flags = 0, // Escrita
			.buffer = &dummy,
			.length = 1
		};

		ret = I2C_TRANSFER(i2c, &msg, 1);

		if (ret == 0) {
			PX4_INFO("ACK recebido com sucesso do SE051!");
		} else {
			PX4_ERR("Falha na escrita I2C: sem ACK (erro %d)", ret);
		}

	}
	
	px4_i2cbus_uninitialize(i2c);
	return (ret == 0) ? 0 : -1;
}
