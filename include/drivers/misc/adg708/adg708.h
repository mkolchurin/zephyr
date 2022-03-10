#ifndef __ADG_708_H__
#define __ADG_708_H__

int adg708_reset(const struct device *dev);
int adg708_select_channel(const struct device *dev, uint32_t channel);


#endif //__ADG_708_H__