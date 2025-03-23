
#include <time.h>
#include <tc_log.h>
#include <qurt_alloc.h>

__attribute__((visibility("default"))) void free(void *ptr)
{
	qurt_free(ptr);
	ptr = 0;
}

__attribute__((visibility("default"))) void *malloc(size_t size)
{
	return qurt_malloc(size);
}

__attribute__((visibility("default"))) void *calloc(size_t nmemb, size_t size)
{
	TC_ERR("Undefined calloc called");
	return (void *) 0;
}

__attribute__((visibility("default"))) void *realloc(void *ptr, size_t size)
{
	TC_ERR("Undefined realloc called");
	return (void *) 0;
}

__attribute__((visibility("default"))) int nanosleep(const struct timespec *req, struct timespec *rem)
{
	TC_ERR("Undefined nanosleep called");
	return -1;
}
