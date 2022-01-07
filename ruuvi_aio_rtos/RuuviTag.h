#ifndef __RUUVI_TAG_H__
#define __RUUVI_TAG_H__

#define  MAX_NBR_RUUVI_TAG  4
#define  RUUVI_LOCATION_LEN 20

typedef struct
{
    String   mac_addr;
    char     location[RUUVI_LOCATION_LEN];
    float    temp_fp;
    uint16_t temperature;
    float    humidity;
    uint16_t pressure;
    int16_t  acc[3];
    uint16_t voltage_power;
    uint16_t voltage;
    uint16_t power;
    uint16_t rssi;
    uint16_t movement;
    uint16_t measurement;
    bool     updated;
} ruuvi_tag_st;


class RuuviTag
{
    public:    
        RuuviTag(void);
        int8_t add(String addr, const char *loc);
        String *get_addr(uint8_t indx);
        char   *get_location(uint8_t indx);
        void   decode_raw_data(String mac_addr, String raw_data, int rssi);
        //bool is_updated(uint8_t indx);
        //void clear_update(uint8_t indx);
        ruuvi_tag_st  ruuvi[MAX_NBR_RUUVI_TAG];
        uint8_t   nbr_of = 0;
   private:
        int8_t    _active_indx = 0;
        String    undef_addr = "xxx";
};
#endif
