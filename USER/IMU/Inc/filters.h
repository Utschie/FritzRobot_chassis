#ifndef __FILTERS_H__
#define __FILTERS_H__
#define STATIC_THRESHHOLD 0.001//��СҲ��Ϊ0.003���������Ư
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