#ifndef __FILTERS_H__
#define __FILTERS_H__
#define STATIC_THRESHHOLD 0.001
#define STATIC_PERIOD 500//unit [ms]

#ifdef __cplusplus
extern "C" {
#endif

void StaticFilter_Init(void);
void StaticFilter(void);
#ifdef __cplusplus
}
#endif

#endif /* __FILTERS__ */