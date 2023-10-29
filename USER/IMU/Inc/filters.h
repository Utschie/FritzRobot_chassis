#ifndef __FILTERS_H__
#define __FILTERS_H__
#define STATIC_THRESHHOLD 0.001//最小也得为0.003·，否则会漂
#define STATIC_PERIOD 500//unit [ms]

#ifdef __cplusplus
extern "C" {
#endif

void StaticFilter_Init(void);
void StaticFilter_x(void);
void StaticFilter_y(void);
void StaticFilter_z(void);
#ifdef __cplusplus
}
#endif

#endif /* __FILTERS__ */