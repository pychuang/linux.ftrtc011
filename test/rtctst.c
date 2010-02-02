#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<sys/ioctl.h>
#include<stdio.h>
#include<linux/rtc.h>
#include<time.h>
#define switch_RTC_IOCTL(fd, tst)  ({			\
	fprintf(stderr,"Testing " #tst ":"); 		\
	if(!ioctl(fd, tst))				\
		fprintf(stderr,"success:\n");	 	\
	else 						\
		perror("failed"); 			\
	;})

#define read_RTC_IOCTL_unsigned(fd, tst) 		\
	({ unsigned t;					\
	fprintf(stderr,"Testing " #tst ":"); 			\
	if(!ioctl(fd, tst,&t))				\
		fprintf(stderr,"success: return %d\n",t); 	\
	else 						\
		perror("failed"); 			\
	t;})

#define set_RTC_IOCTL_unsigned(fd, tst, l) ({ int r;\
	fprintf(stderr, "Testing %s:", #tst ); \
	if(!(r=ioctl(fd, tst,l))) { \
		fprintf(stderr, "success:\n"); \
	} else perror("failed");r;})

#define read_RTC_IOCTL_tm(fd, tst) 			\
	({struct rtc_time t;				\
	fprintf(stderr, "Testing " #tst ":"); 			\
	if(!ioctl(fd, tst,&t)) {			\
		fprintf(stderr, "success:\tdate=%d/%d/%d,", 	\
			1900+t.tm_year, t.tm_mon+1, t.tm_mday); \
		fprintf(stderr, "time=%d:%d:%d,", 		\
			t.tm_hour, t.tm_min, t.tm_sec); 	\
		fprintf(stderr, "DaylightSaving=%s",t.tm_isdst?"Y":"N");	\
		fprintf(stderr, "\n");				\
	}						\
	else 						\
		perror("failed:"); 			\
	t;})

#define set_RTC_IOCTL_tm(fd, tst, t) 			\
	({						\
	fprintf(stderr, "Testing " #tst ":"); 		\
	if(ioctl(fd, tst,&t))				\
		perror("failed:"); 			\
	else						\
		fprintf(stderr,"success:\n");})


int main()
{
	unsigned long l;
	int fd;
	struct rtc_time t;
	int i;

	printf("RTC Test Program via Linux IOCTL interface\n");
	printf("By I-Jui Sung <ijsung@faraday-tech.com>\n");

	fd=open("/dev/rtc0",O_RDONLY);
	if(!fd) {
		printf("Can not open /dev/rtc0\n");
		return -1;
	}

	/* Read time*/
	t=read_RTC_IOCTL_tm(fd, RTC_RD_TIME);

	/* Set time to 5 min ahead*/
	t.tm_min=(t.tm_min+5)%60;
	if(t.tm_min<5) t.tm_hour++;
	t.tm_year=105;
	set_RTC_IOCTL_tm(fd, RTC_SET_TIME, t);

	/* Read time again*/
	t=read_RTC_IOCTL_tm(fd, RTC_RD_TIME);

#if 0	/* ratbert: not supported */
	/*Read EPOCH*/
	l=read_RTC_IOCTL_unsigned(fd, RTC_EPOCH_READ);

	/*Set EPOCH*/
	if(!set_RTC_IOCTL_unsigned(fd, RTC_EPOCH_SET,2000)) {
		/*Read EPOCH*/
		read_RTC_IOCTL_unsigned(fd, RTC_EPOCH_READ);
		/*Set EPOCH back*/
		set_RTC_IOCTL_unsigned(fd, RTC_EPOCH_SET,l);
	}
#endif

	/*Test Alarm after 10 seconds*/
	t=read_RTC_IOCTL_tm(fd, RTC_RD_TIME);
	t.tm_sec=(t.tm_sec+10)%60;
	if(t.tm_sec<10) t.tm_min++;
	set_RTC_IOCTL_tm(fd, RTC_ALM_SET, t);
	read_RTC_IOCTL_tm(fd, RTC_ALM_READ);

	/*enable alarm*/
	switch_RTC_IOCTL(fd, RTC_AIE_ON);
	if(read(fd,&l, sizeof(unsigned long))==-1)
		perror("Read RTC failed");
	else 
		fprintf(stderr,"done(%d)\n",l);

	switch_RTC_IOCTL(fd, RTC_AIE_OFF);

	/* Read IRQ rate */
	read_RTC_IOCTL_unsigned(fd, RTC_IRQP_READ);

	/* Set IRQ rate */
	set_RTC_IOCTL_unsigned(fd, RTC_IRQP_SET, 2);

	/* test periodic alarm */
	switch_RTC_IOCTL(fd, RTC_PIE_ON);
	for (i = 0; i < 5; i++) {
		if (read(fd,&l, sizeof(unsigned long))==-1) {
			perror("read RTC failed");
		} else {
			fprintf(stderr, "%d time: %d\n", i, l);
		}
	}
	switch_RTC_IOCTL(fd, RTC_PIE_OFF);

	return 0;
}
