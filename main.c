#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <termio.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>

#include "uinput.h"
#include "inih/ini.h"

typedef struct
{
    uint8_t address_l;
    uint8_t address_h;
    uint8_t power_key;
    uint8_t flags;
} avr_config;

typedef struct
{
    char name[256];
    char serial[256];
    char uinput[256];
    uint32_t period;
    uint32_t delay;
    bool reset;         /* request reset at start */
    bool debug;         /* enable debug */
    bool power_confirm; /* power confirmation */
    bool check_avr_cfg;

    avr_config avr_cfg;
    uint16_t key_map[IR_MAX_KEYS_NUM];
} configuration;

typedef struct {
    uint8_t address_l;
    uint8_t address_h;
    uint8_t power_key;
    uint8_t flags;
    uint8_t hash;
} avr_scfg_t;


typedef union {
    avr_scfg_t s;
    uint8_t data[0];
} avr_cfg_t;

int usleep(unsigned int);

static void config_error(const char *section, const char *name, const char *value)
{
    printf("config_handler wrong %s item -> name[%s] value[%s]\n", section, name, value);
}

static int config_handler(void *ptr, const char *section, const char *name, const char *value)
{
    configuration *pconfig = (configuration*)ptr;

    if (0 == strcmp(section, "avr_cfg"))
    {
        if (0 == strcmp(name, "address_l"))
        {
            if (0 == sscanf(value, "%hhu", &(pconfig->avr_cfg.address_l)))
            {
                config_error(section, name, value);
                return 0;
            }
        }

        else if (0 == strcmp(name, "address_h"))
        {
            if (0 == sscanf(value, "%hhu", &(pconfig->avr_cfg.address_h)))
            {
                config_error(section, name, value);
                return 0;
            }
        }

        else if (0 == strcmp(name, "power_key"))
        {
            if (0 == sscanf(value, "%hhu", &(pconfig->avr_cfg.power_key)))
            {
                config_error(section, name, value);
                return 0;
            }
        }

        else if (0 == strcmp(name, "flags"))
        {
            if (0 == sscanf(value, "%hhu", &(pconfig->avr_cfg.flags)))
            {
                config_error(section, name, value);
                return 0;
            }
        }

    }
    else if (0 == strcmp(section, "config"))
    {
        if (0 == strcmp(name, "serial"))
        {
            strncpy(pconfig->serial, value, sizeof(pconfig->serial)-1);
        }
        else if (0 == strcmp(name, "name"))
        {
            strncpy(pconfig->name, value, sizeof(pconfig->name)-1);
        }
        else if (0 == strcmp(name, "uinput"))
        {
            strncpy(pconfig->uinput, value, sizeof(pconfig->uinput)-1);
        }
        else if (0 == strcmp(name, "period"))
        {
            if (0 == sscanf(value, "%u", &(pconfig->period)))
            {
                config_error(section, name, value);
                return 0;
            }
        }
        else if (0 == strcmp(name, "delay"))
        {
            if (0 == sscanf(value, "%u", &(pconfig->delay)))
            {
                config_error(section, name, value);
                return 0;
            }
        }
        else if (0 == strcmp(name, "reset"))
        {
            if (0 != strcmp(value, "1") && 0 != strcmp(value, "0"))
            {
                config_error(section, name, value);
                return 0;
            }
            pconfig->reset = value[0] == '1' ? true : false;
        }
        else if (0 == strcmp(name, "debug"))
        {
            if (0 != strcmp(value, "1") && 0 != strcmp(value, "0"))
            {
                config_error(section, name, value);
                return 0;
            }
            pconfig->debug = value[0] == '1' ? true : false;
        }
        else if (0 == strcmp(name, "power_confirm"))
        {
            if (0 != strcmp(value, "1") && 0 != strcmp(value, "0"))
            {
                config_error(section, name, value);
                return 0;
            }
            pconfig->power_confirm = value[0] == '1' ? true : false;
        }
        else if (0 == strcmp(name, "check_avr_cfg"))
        {
            if (0 != strcmp(value, "1") && 0 != strcmp(value, "0"))
            {
                config_error(section, name, value);
                return 0;
            }
            pconfig->check_avr_cfg = value[0] == '1' ? true : false;
        }
    }
    else if (0 == strcmp(section, "keymap"))
    {
        int32_t key_code = -1;
        sscanf(name, "%d", &key_code);
        if (key_code >= 0 && key_code < IR_MAX_KEYS_NUM)
        {
            if (0 == sscanf(value, "%hu", pconfig->key_map + key_code))
            {
                config_error(section, name, value);
                return 0;
            }
        }
        else
        {
            config_error(section, name, value);
            return 0;
        }
    }
    else
    {
        return 0;  /* unknown section/name, error */
    }
    return 1;
}

static int output_rts(int fd, int val)
{
    int retorno;
    int argumento = TIOCM_RTS;
    if(val)
    {
        retorno = ioctl(fd,TIOCMBIS,&argumento);
    }
    else
    {
        retorno = ioctl(fd,TIOCMBIC,&argumento);
    }
    return retorno;
}

static int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }

    // give time to, data to arrive in to the read buffer
    sleep(1);
    tcflush(fd, TCIOFLUSH);

    output_rts(fd, 0);
    return 0;
}

int main(int argc, char *argv[])
{
    ssize_t read_ret = 0;
    int fd = -1 ;
    int uinputfd = -1;
    char buf[36] = "";
    int rlen = 0;
    char ch = '\0';
    int32_t i = 0;
    char *config_file = "/etc/srcd.ini";

    configuration config;

    memset(&config, 0x00, sizeof(config));
    for (i=0; i<IR_MAX_KEYS_NUM; ++i)
    {
        config.key_map[i] = (uint16_t)-1; // set all event code's as UNKNOWN
    }


    if (argc > 1)
    {
        if (0 == strcmp(argv[1], "--help"))
        {
            printf("Usage v2026.01.11: \n");
            printf("\t%s [path to config file] \n", argv[0]);
            printf("\t\t path to config file - default: %s\n", config_file);
            exit(0);
        }
        config_file = argv[1];
    }

    if (ini_parse(config_file, config_handler, &config) < 0)
    {
        printf("Can't load \"%s\"\n", config_file);
        return 1;
    }

    if ('\0' == config.name[0])
    {
        strcpy(config.name, "ATtiny2313-uinput");
    }

    if ('\0' == config.name[0])
    {
        strcpy(config.name, "/dev/uinput");
    }

    if ('\0' == config.serial[0])
    {
        printf("Please set serial in the config file!\n");
        return 2;
    }

    fd = open(config.serial, O_RDWR | O_NOCTTY | O_SYNC);
    while (fd < 0)
    {
        printf("Error opening %s: %s\n", config.serial, strerror(errno));
        sleep(1);
        continue;
    }

    uinputfd = setup_uinputfd(config.uinput, config.name, config.key_map, config.delay, config.period);

    /*baudrate B9600, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B9600); //B1200

    if (config.power_confirm)
    {
        /* confirm at start */
        write(fd, "CCC", 3);
    }

    if (config.reset)
    {
        /* request reset at start */
        write(fd, ".RsT", 4);
    }

    if (1 || uinputfd > -1)
    {
        while (1) 
        {
            read_ret = read(fd, &ch, 1);
            if (read_ret == -1)
            {
                if (errno == EINTR)         /* Interrupted --> restart read() */
                    continue;
                else
                    return -1;              /* Some other error */
            }
            else if (read_ret == 0)         /* EOF */
            {
                printf("EOF\n");
                break;
            } 
            else
            {
                if (ch == '\n')
                {
                    if (rlen > 0)
                    {
                        buf[rlen] = '\0';
                        if (config.debug)
                        {
                            printf("%s\n", buf);
                        }

                        if (0 == strncmp(buf, "CfK", 3) || 0 == strncmp(buf, "CfO", 3))
                        {
                            write(fd, ".RsT", 4);
                            config.check_avr_cfg = false;
                        }
                        else if (0 == strncmp(buf, "CfS", 3))
                        {
                            int i = 0;
                            avr_cfg_t avr_cfg;
                            avr_cfg.s.address_l = config.avr_cfg.address_l;
                            avr_cfg.s.address_h = config.avr_cfg.address_h;
                            avr_cfg.s.power_key = config.avr_cfg.power_key;
                            avr_cfg.s.flags = config.avr_cfg.flags;
                            avr_cfg.s.hash = 0xAA;
                            for (i = 0; i < sizeof(avr_cfg)-1; ++i)
                            {
                                avr_cfg.s.hash ^= avr_cfg.data[i];
                            }

                            write(fd, avr_cfg.data, sizeof(avr_cfg));
                        }
                        else if (0 == strncmp(buf, "StArT", 5))
                        {
                            avr_cfg_t avr_cfg;
                            size_t rsize = 0;
                            while (rsize < sizeof(avr_cfg))
                            {
                                rsize += read(fd, avr_cfg.data + rsize, sizeof(avr_cfg) - rsize);
                                usleep(100);
                            }

                            if (sizeof(avr_cfg) == rsize)
                            {
                                if (config.debug)
                                {
                                    uint8_t hash = 0xAA;
                                    int i = 0;
                                    for (i = 0; i < sizeof(avr_cfg)-1; ++i)
                                    {
                                        hash ^= avr_cfg.data[i];
                                    }
                            
                                    printf("AVR config received:\n");
                                    printf("\taddress_l:%hhu\n", avr_cfg.s.address_l);
                                    printf("\taddress_h:%hhu\n", avr_cfg.s.address_h);
                                    printf("\tpower_key:%hhu\n", avr_cfg.s.power_key);
                                    printf("\tflags:%hhu\n", avr_cfg.s.flags);
                                    printf("\thash:%hhu\n", avr_cfg.s.hash);
                                    printf("\tcalc hash:%hhu\n", hash);
                                }
                                if (config.check_avr_cfg)
                                {
                                    if (config.avr_cfg.address_l != avr_cfg.s.address_l ||
                                        config.avr_cfg.address_h != avr_cfg.s.address_h ||
                                        config.avr_cfg.power_key != avr_cfg.s.power_key ||
                                        config.avr_cfg.flags != avr_cfg.s.flags)
                                    {
                                        if (config.debug)
                                        {
                                            printf("AVR has wrong config try to set correct one!\n");
                                            write(fd, ".CfG", 4);
                                        }
                                    }
                                    //write(fd, "RsT", 3);
                                }
                            }
                            else
                                printf("WRONG CONFIG SIZE: %zu, EXPECTED: %zu\n", rsize, sizeof(avr_cfg));
                        }
                        else if (0 == strncmp(buf, "CoLd", 4))
                        {
                            int fdCold = open("/tmp/srcd_cold_boot", O_CREAT | O_EXCL, 0644);
                            if (fdCold >= 0) {
                                close(fdCold);
                            }
                        }
                        else 
                        {
                            process_key_message(uinputfd, buf, config.power_confirm ? fd : -1);
                        }
                    }
                    rlen = 0;
                }
                else
                {
                    if (rlen < (sizeof(buf)-2))
                        buf[rlen++] = ch;
                    else
                        printf("Buffer overflow - skip char [%c]\n", ch);
                }
            }
        }
    }
    else
    {
        printf("setup_uinputfd failed\n");
    }
    return 0;
}