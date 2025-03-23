#include <tc_platform_common/log.h>
#include <tc_platform_common/app.h>

extern "C" __EXPORT int hello_main(int argc, char *argv[]);
int hello_main(int argc, char *argv[])
{
	TC_INFO("Hello, I am a dynamically loaded module.");

	TC_INFO("Argv:");

	for (int i = 0; i < argc; ++i) {
		TC_INFO("  %d: %s", i, argv[i]);
	}

	return 0;
}
