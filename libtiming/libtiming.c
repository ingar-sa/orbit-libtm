
#include <stdio.h>
#include <time.h> // for nanosleep() & struct timespec
#include <libtiming.h>
#include <unistd.h> // for close(), read(), write()
#include <string.h> 
// #include <ds1307/ds1307.h> // Framsat library
#include <stdlib.h> // for malloc()
#include <stdint.h> // for uint_t


////////////////////////////////////////////////////////////////////////////////
// Public functions
////////////////////////////////////////////////////////////////////////////////

void delay_ms(int ms)
{
    struct timespec req;
    req.tv_sec  = ms / 1000;
    req.tv_nsec = (ms % 1000) * 1000000;
    nanosleep(&req, 0);
}

void timeadd(struct timeval* tmvl1, struct timeval* tmvl2,
             struct timeval* tmvl_sum)
{
    tmvl_sum->tv_sec  = tmvl1->tv_sec + tmvl2->tv_sec;
    tmvl_sum->tv_usec = tmvl1->tv_usec + tmvl2->tv_usec;

    if (tmvl_sum->tv_usec > 999999)
    {
        tmvl_sum->tv_usec = tmvl_sum->tv_usec - 1000000;
        tmvl_sum->tv_sec++;
    }
}

void timediff(struct timeval* tmvl1, struct timeval* tmvl2,
              struct timeval* tmvl_diff)
{
    tmvl_diff->tv_sec  = tmvl1->tv_sec - tmvl2->tv_sec;
    tmvl_diff->tv_usec = tmvl1->tv_usec - tmvl2->tv_usec;

    if (tmvl_diff->tv_usec < 0)
    {
        tmvl_diff->tv_usec = tmvl_diff->tv_usec + 1000000;
        tmvl_diff->tv_sec--;
    }
}

int timediff_ms(struct timeval* tmvl1, struct timeval* tmvl2)
{
    int diff_s  = tmvl1->tv_sec - tmvl2->tv_sec;
    int diff_us = tmvl1->tv_usec - tmvl2->tv_usec;

    return 1000 * diff_s + diff_us / 1000;
}

void date_to_unix_time(struct Date_RTC* date_rtc, struct timeval* tmvl)
{
    unsigned char daysInMonth[12]
        = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    int isCurrentYearLeap = ((date_rtc->years % 4)) == 0;
    if (isCurrentYearLeap)
    {
        daysInMonth[1] = 29;
    }

    int returnSeconds = date_rtc->seconds;
    returnSeconds += 60 * date_rtc->minutes + 60 * 60 * date_rtc->hours
        + 24 * 60 * 60 * (date_rtc->days - 1);

    for (int i = 0; i < (date_rtc->months - 1); i++)
    {
        returnSeconds += (int)daysInMonth[i] * 24 * 60 * 60;
    }

    for (int i = 0; i < date_rtc->years; i++)
    {
        int isYearLeap = (i % 4) == 0;
        if (isYearLeap)
        {
            returnSeconds += 366 * 24 * 60 * 60;
        }
        else
        {
            returnSeconds += 365 * 24 * 60 * 60;
        }
    }

    tmvl->tv_sec  = returnSeconds + UNIX_TIME_AT_00_00_00_01_01_2000;
    tmvl->tv_usec = 0;
}

void unix_time_to_date(struct timeval* tmvl, struct Date_RTC* date_rtc_out)
{
    int scs = tmvl->tv_sec - UNIX_TIME_AT_00_00_00_01_01_2000;

    int mns = scs / 60;
    int hrs = mns / 60;
    int dys = hrs / 24;

    date_rtc_out->seconds = scs - 60 * mns;
    date_rtc_out->minutes = mns - 60 * hrs;
    date_rtc_out->hours   = hrs - 24 * dys;

    date_rtc_out->years = 0;
    int x               = dys; // here, secs contains amount of days since 2000
    while (1)
    {
        if ((date_rtc_out->years % 4) == 0)
        {
            x = x - 366;
        }
        else
        {
            x = x - 365;
        }

        if (x < 0)
        {
            break;
        }
        else
        {
            date_rtc_out->years++;
            dys = x;
        }
    }

    // secs at this point contains days since beginning of most recent year

    unsigned char daysInMonth[12]
        = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    if ((date_rtc_out->years % 4) == 0)
    {
        daysInMonth[1] = 29;
    }

    date_rtc_out->months = 1;
    x                    = dys;
    for (int i = 0; i < 12; i++)
    {
        x = x - daysInMonth[i];
        if (x < 0)
        {
            break;
        }
        else
        {
            date_rtc_out->months++;
            dys = x;
        }
    }

    // secs at this point contains days since beginning of most recent month

    date_rtc_out->days = dys + 1;
}

int get_RTC_date(int fd_i2c, struct Date_RTC* date_rtc)
{

    unsigned char data[1 + 7] = { DS1307_ADDR, 0 };

    if (write(fd_i2c, data, 2) != 2)
    {
        return -1;
    }

    if (read(fd_i2c, data, 8) != 8)
    {
        return -2;
    }

    date_rtc->seconds = 10 * ((data[1] >> 4) & 7) + (data[1] & 0x0F);
    date_rtc->minutes = 10 * (data[2] >> 4) + (data[2] & 0x0F);
    date_rtc->hours   = 10 * ((data[3] >> 4) & 3) + (data[3] & 0x0F);
    date_rtc->days    = 10 * (data[5] >> 4) + (data[5] & 0x0F);
    date_rtc->months  = 10 * (data[6] >> 4) + (data[6] & 0x0F);
    date_rtc->years   = 10 * (data[7] >> 4) + (data[7] & 0x0F);

    return 0;
}

int set_RTC_date(int fd_i2c, struct Date_RTC* date_rtc)
{
    // Reconstruct register values from struct
    unsigned char seconds_reg = ((date_rtc->seconds / 10) << 4)

                              | ((date_rtc->seconds % 10)     );

    unsigned char minutes_reg = ((date_rtc->minutes / 10) << 4)
                              | ((date_rtc->minutes % 10)     );
    
    unsigned char hours_reg   = ((date_rtc->hours   / 20) << 5)
                              | ((date_rtc->hours   / 10) << 4)
                              | ((date_rtc->hours   % 10)     );
    
    unsigned char date_reg    = ((date_rtc->days    / 10) << 4)
                              | ((date_rtc->days    % 10)     );
    
    unsigned char month_reg   = ((date_rtc->months  / 10) << 4)
                              | ((date_rtc->months  % 10)     );
    
    unsigned char year_reg    = ((date_rtc->years   / 10) << 4)
                              | ((date_rtc->years   % 10)     );

    // Day register is not represented in the struct
    unsigned char data[2 + 7] = { DS1307_ADDR, 0, seconds_reg, minutes_reg, hours_reg, 0,
                                                  date_reg, month_reg, year_reg };

    if (write(fd_i2c, data, 9) != 9)
    {
        return -1;
    }
    else 
    {
        return 0;
    }
}

int get_RTC_unix_time(int fd_i2c, struct timeval* tmvl)
{

    struct Date_RTC date_rtc;
    int             retval = get_RTC_date(fd_i2c, &date_rtc);
    if (retval < 0)
    {
        return retval;
    }

    date_to_unix_time(&date_rtc, tmvl);

    return 0;
}

void unix_time_to_buffer(struct timeval* tmvl, char* buffer){
    int scs = tmvl -> tv_sec;
    int ms = tmvl -> tv_usec/1000;

    if(ms < 10){
        sprintf(buffer, "%i.00%i", scs, ms);
    }else if(ms < 100){
        sprintf(buffer, "%i.0%i", scs, ms);
    }else{
        sprintf(buffer, "%i.%i", scs, ms);
    }
}

int set_RTC_unix(int fd_i2c, unsigned int unix_time)
{
    struct timeval unix_tmvl = {.tv_sec = unix_time};

    struct Date_RTC current_date;
    unix_time_to_date(&unix_tmvl, &current_date);

    char unix_string[24] = "";
    
    sprintf(&(unix_string[0]), "20");

    if (current_date.years < 10)
        sprintf(&(unix_string[2]), "0%u-", current_date.years);
    sprintf(&(unix_string[2]), "%u-", current_date.years);

    if (current_date.months < 10)
        sprintf(&(unix_string[5]), "0%u-", current_date.months);
    sprintf(&(unix_string[5]), "%u-", current_date.months);

    if (current_date.days < 10)
        sprintf(&(unix_string[8]), "0%u", current_date.days);
    sprintf(&(unix_string[8]), "%u", current_date.days);

    // Day of the week, 1 = monday, 7 = sunday. This doesn't matter most likely
    sprintf(&(unix_string[10]), " 1 ");

    if (current_date.hours < 10)
        sprintf(&(unix_string[13]), "0%u:", current_date.hours);
    sprintf(&(unix_string[13]), "%u:", current_date.hours);

    if (current_date.minutes < 10)
        sprintf(&(unix_string[16]), "0%u:", current_date.minutes);
    sprintf(&(unix_string[16]), "%u:", current_date.minutes);

    if (current_date.seconds < 10)
        sprintf(&(unix_string[19]), "0%u", current_date.minutes);
    sprintf(&(unix_string[19]), "%u", current_date.minutes);

    //printf("time in string format: %s\n", unix_string);
    
    return DS1307_SetTimeAsString(fd_i2c, DS1307_ADDR, unix_string);
}

void print_unix_time(struct timeval* tmvl)
{
    int msc = tmvl->tv_usec / 1000;
    int scs = tmvl->tv_sec - UNIX_TIME_AT_00_00_00_01_01_2000;
    int mns = scs / 60;
    int hrs = mns / 60;
    int dys = hrs / 24;

    scs = scs - 60 * mns;
    mns = mns - 60 * hrs;
    hrs = hrs - 24 * dys;

    if (dys < 1)
    {
        printf("[0000 ");
    }
    else if (dys < 10)
    {
        printf("[000%i ", dys);
    }
    else if (dys < 100)
    {
        printf("[00%i ", dys);
    }
    else if (dys < 1000)
    {
        printf("[0%i ", dys);
    }
    else
    {
        printf("%i ", dys);
    }

    if (hrs < 1)
    {
        printf("00:");
    }
    else if (hrs < 10)
    {
        printf("0%i:", hrs);
    }
    else
    {
        printf("%i:", hrs);
    }

    if (mns < 1)
    {
        printf("00:");
    }
    else if (mns < 10)
    {
        printf("0%i:", mns);
    }
    else
    {
        printf("%i:", mns);
    }

    if (scs < 1)
    {
        printf("00.");
    }
    else if (scs < 10)
    {
        printf("0%i.", scs);
    }
    else
    {
        printf("%i.", scs);
    }

    if (msc < 1)
    {
        printf("000]");
    }
    else if (msc < 10)
    {
        printf("00%i] ", msc);
    }
    else if (msc < 100)
    {
        printf("0%i] ", msc);
    }
    else
    {
        printf("%i] ", msc);
    }
}

void print_RTC_date(struct Date_RTC* date_rtc)
{
    // year
    if (date_rtc->years < 10)
    {
        printf("200%i ", date_rtc->years);
    }
    else
    {
        printf("20%i ", date_rtc->years);
    }

    // month
    if (date_rtc->months < 10)
    {
        printf("0%i ", date_rtc->months);
    }
    else
    {
        printf("%i ", date_rtc->months);
    }

    // day
    if (date_rtc->days < 10)
    {
        printf("0%i ", date_rtc->days);
    }
    else
    {
        printf("%i ", date_rtc->days);
    }

    // hours
    if (date_rtc->hours < 10)
    {
        printf("0%i:", date_rtc->hours);
    }
    else
    {
        printf("%i:", date_rtc->hours);
    }

    // minutes
    if (date_rtc->minutes < 10)
    {
        printf("0%i:", date_rtc->minutes);
    }
    else
    {
        printf("%i:", date_rtc->minutes);
    }

    // secnonds
    if (date_rtc->seconds < 10)
    {
        printf("0%i\n", date_rtc->seconds);
    }
    else
    {
        printf("%i\n", date_rtc->seconds);
    }
}

void printTime_old(unsigned int t)
{

    int           t_     = t / 10;
    unsigned char _100ms = t - t_ * 10;
    t                    = t_ / 60;
    unsigned char s      = t_ - t * 60;
    t_                   = t / 60;
    unsigned char m      = t - t_ * 60;
    t                    = t_ / 24;
    unsigned char h      = t_ - t * 24;
    unsigned char d      = t;

    if (d < 10)
    {
        printf("[00%i ", d);
    }
    else if (d < 100)
    {
        printf("[0%i ", d);
    }
    else
    {
        printf("[%i ", d);
    }

    if (h < 10)
    {
        printf("0%i:", h);
    }
    else
    {
        printf("%i:", h);
    }

    if (m < 10)
    {
        printf("0%i:", m);
    }
    else
    {
        printf("%i:", m);
    }

    if (s < 10)
    {
        printf("0%i.", s);
    }
    else
    {
        printf("%i.", s);
    }

    printf("%i] ", _100ms);
}

int printTimeToBuffer_old(unsigned int t, char* buffer)
{
    int           t_     = t / 10;
    unsigned char _100ms = t - t_ * 10;
    t                    = t_ / 60;
    unsigned char s      = t_ - t * 60;
    t_                   = t / 60;
    unsigned char m      = t - t_ * 60;
    t                    = t_ / 24;
    unsigned char h      = t_ - t * 24;
    unsigned char d      = t;

    // 0 1 2 3 4 5 6 7 8 91011121314151617
    //[ 0 2 6   1 5 : 2 9 : 0 3 . 5 ]
    // [ddd hh:mm:ss.ms

    if (d < 10)
    {
        sprintf(&(buffer[0]), "[00%i ", d);
    }
    else if (d < 100)
    {
        sprintf(&(buffer[0]), "[0%i ", d);
    }
    else
    {
        sprintf(&(buffer[0]), "[%i ", d);
    }

    if (h < 10)
    {
        sprintf(&(buffer[5]), "0%i:", h);
    }
    else
    {
        sprintf(&(buffer[5]), "%i:", h);
    }

    if (m < 10)
    {
        sprintf(&(buffer[8]), "0%i:", m);
    }
    else
    {
        sprintf(&(buffer[8]), "%i:", m);
    }

    if (s < 10)
    {
        sprintf(&(buffer[11]), "0%i.", s);
    }
    else
    {
        sprintf(&(buffer[11]), "%i.", s);
    }

    sprintf(&(buffer[14]), "%i] ", _100ms);

    return 0;
}