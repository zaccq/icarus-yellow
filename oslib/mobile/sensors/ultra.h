#ifndef ULTRA_H
#define ULTRA_H

void init_untra(const struct device *dev_gpio);

struct ultra_reading {
    uint8_t dist[2];
};

extern struct k_msgq ultra_q;

#endif