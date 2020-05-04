/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Simple user application.
 *
 * Copyright (c) 2016-2019, LEAPS. All rights reserved.
 *
 */


#include "dwm.h"
#include <stdio.h>
#include <math.h>

/* Thread priority */
#ifndef THREAD_APP_PRIO
#define THREAD_APP_PRIO	20
#endif /* THREAD_APP_PRIO */

/* Thread stack size */
#ifndef THREAD_APP_STACK_SIZE
#define THREAD_APP_STACK_SIZE	(3 * 1024)
#endif /* THREAD_APP_STACK_SIZE */

#define APP_ERR_CHECK(err_code)	\
do {							\
	if ((err_code) != DWM_OK)	\
		printf("err: line(%u) code(%u)", __LINE__, (err_code));\
} while (0)						\

#define MSG_INIT	\
	"\n\n"	\
	"App   :  dwm-simple\n"	\
	"Built :  " __DATE__ " " __TIME__ "\n"	\
	"\n"

/**
 * Event callback
 *
 * @param[in] p_evt  Pointer to event structure
 */
void on_dwm_evt(dwm_evt_t *p_evt)
{
	int len;
	int i;

	switch (p_evt->header.id) {
	/* New location data */
	case DWM_EVT_LOC_READY:
		printf("\nT:%lu ", dwm_systime_us_get());
		if (p_evt->loc.pos_available) {
			printf("POS:[%ld,%ld,%ld,%u] ", p_evt->loc.pos.x,
					p_evt->loc.pos.y, p_evt->loc.pos.z,
					p_evt->loc.pos.qf);
		} else {
			printf("POS:N/A ");
		}

		/*for (i = 0; i < p_evt->loc.anchors.dist.cnt; ++i) {
			printf("DIST%d:", i);

			printf("0x%04X", (unsigned int)(p_evt->loc.anchors.dist.addr[i] & 0xffff));
			if (i < p_evt->loc.anchors.an_pos.cnt) {
				printf("[%ld,%ld,%ld]",
						p_evt->loc.anchors.an_pos.pos[i].x,
						p_evt->loc.anchors.an_pos.pos[i].y,
						p_evt->loc.anchors.an_pos.pos[i].z);
			}

			printf("=[%lu,%u] ", p_evt->loc.anchors.dist.dist[i],
					p_evt->loc.anchors.dist.qf[i]);
		}*/
		printf("\n");
		break;

	case DWM_EVT_USR_DATA_READY:
		len = p_evt->header.len - sizeof(dwm_evt_hdr_t);
		if (len <= 0)
			break;

		printf("iot received, len=%d:", len);
		for (i = 0; i < len; ++i) {
			printf(" %02X", p_evt->usr_data[i]);
		}
		break;

	case DWM_EVT_USR_DATA_SENT:
		printf("iot sent\n");
		break;

	case DWM_EVT_BH_INITIALIZED_CHANGED:
		printf("uwbmac: backhaul = %d\n", p_evt->bh_initialized);
		break;

	case DWM_EVT_UWBMAC_JOINED_CHANGED:
		printf("uwbmac: joined = %d\n", p_evt->uwbmac_joined);
		break;

	default:
		break;
	}
}

/*get x,y position using distance


struct Point threePoints(int *dis, struct Point *ps){
        struct Point p;
        p.x=0;
        p.y=0;
        if (dis==NULL || ps==NULL)
                return p;
        for (int j=0; j<3; ++j){
                if (dis[j]<0)
                        printf("There is an error with the distance!");
                for (int k=j+1;k<3;++k){
                        printf("debug:%d\n",dis[j]);
                        float p2p= (float)sqrt((ps[j].x-ps[k].x)*(ps[j].x-ps[k].x)+(ps[j].y-ps[k].y)*(ps[j].y-ps[k].y));
                        if(dis[j]+dis[k]<=p2p){
                                p.x +=ps[j].x+(ps[k].x-ps[j].x)*dis[j]/(dis[j]+dis[k]);
                                //printf("can you see me!!!!");
                                p.y +=ps[j].y+(ps[k].y-ps[j].y)*dis[j]/(dis[j]+dis[k]);}
                        else{
                                float dr=p2p/2+(dis[j]*dis[j]+dis[k]*dis[k])/(2*p2p);
                                p.x += ps[j].x+(ps[k].x-ps[j].x)*dr/p2p;
                                //printf("p.x value is : %d",p.x);
                                //printf("look here!!!!");
                                p.y += ps[j].y+(ps[k].y-ps[j].y)*dr/p2p;
                        }
                }
        }
        p.x /=3;
        p.y /=3;
        double z,a_p,b_p,c_p,cos1,cos2;
        z=dis[0]*dis[0]-((3600*3600+dis[0]*dis[0]-dis[2]*dis[2])/(3600*2))*((3600*3600+dis[0]*dis[0]-dis[2]*dis[2])/(3600*2))-((2400*2400+dis[0]*dis[0]-dis[1]*dis[1])/(2400*2))*((2400*2400+dis[0]*dis[0]-dis[1]*dis[1])/(2400*2));
        printf("z axies is : %f",z);
        z=0;
        a_p=sqrt(dis[0]*dis[0]-z);
        b_p=sqrt(dis[1]*dis[1]-z);
        c_p=sqrt(dis[2]*dis[2]-z);
        cos1=(3600*3600+a_p*a_p-c_p*c_p)/(2*3600*a_p);
        printf("cos1:%f\n",cos1);
        p.x=cos1*a_p;
        cos2=(2400*2400+a_p*a_p-b_p*b_p)/(2*2400*a_p);
        printf("cos2: %f\n",cos2);
        p.y=cos2*a_p;

        
        printf("Calculate position is [%d,%d]", p.x, p.y);
}*/
/**
 * Application thread
 *
 * @param[in] data  Pointer to user data
 */
void app_thread_entry(uint32_t data)
{
	dwm_cfg_t cfg;
	uint8_t i2cbyte;
        uint8_t i2cbyteXH;
        uint8_t i2cbyteXL;
        int16_t i2cbyteX;
        uint8_t i2cbyteYH;
        uint8_t i2cbyteYL;
        int16_t i2cbyteY;
        uint8_t i2cbyteZH;
        uint8_t i2cbyteZL;
        int16_t i2cbyteZ;
        float x,y,z;




	dwm_evt_t evt;
	int rv,i;
	uint8_t label[DWM_LABEL_LEN_MAX];
	uint8_t label_len = DWM_LABEL_LEN_MAX;
        dwm_pos_t pos;
        dwm_loc_data_t loc;
        

	/* Initial message */
	printf(MSG_INIT);

	/* Get node configuration */
	APP_ERR_CHECK(dwm_cfg_get(&cfg));

	/* Update rate set to 1 second, stationary update rate set to 5 seconds */
	APP_ERR_CHECK(dwm_upd_rate_set(10, 10));

	/* Sensitivity for switching between stationary and normal update rate */
	APP_ERR_CHECK(dwm_stnry_cfg_set(DWM_STNRY_SENSITIVITY_NORMAL));

	/* Register event callback */
	dwm_evt_listener_register(
			DWM_EVT_LOC_READY | DWM_EVT_USR_DATA_READY |
			DWM_EVT_BH_INITIALIZED_CHANGED |
			DWM_EVT_UWBMAC_JOINED_CHANGED, NULL);

        /* Test the accelerometer */
        i2cbyte = 0x57;
        rv = dwm_i2c_write(0x20, &i2cbyte, 1, true);
	i2cbyte = 0x0f;
	rv = dwm_i2c_write(0x33 >> 1, &i2cbyte, 1, true);

	if (rv == DWM_OK) {
		rv = dwm_i2c_read(0x33 >> 1, &i2cbyte, 1);

		if (rv == DWM_OK) {
			printf("Accelerometer chip ID: %u\n", i2cbyte);
		} else {
			printf("i2c: read failed (%d)\n", rv);
		}
	} else {
		printf("i2c: write failed (%d)\n", rv);
	}

	rv = dwm_label_read(label, &label_len);

	if (rv == DWM_OK) {
		printf("LABEL(len=%d):", label_len);
		for (rv = 0; rv < label_len; ++rv) {
			printf(" %02x", label[rv]);
		}
		printf("\n");
	} else {
		printf("can't read label len=%d, error %d\n", label_len, rv);
	}
        /*get initial velocity*/
        int a, b, c, d, e, f;
         rv = dwm_loc_get(&loc);
         a=loc.pos.x;
         b=loc.pos.y;
         c=loc.pos.z;


	while (1) {
/*get last distance to the anchors and the position*/
               rv = dwm_loc_get(&loc);
               printf("0");
 
        if (0 == rv) {
                if (loc.pos_available) {
                        printf("[%ld,%ld,%ld,%u] ", loc.pos.x, loc.pos.y, loc.pos.z,loc.pos.qf);
                        printf("\n");


                        //printf("Calculate position is [%d,%d]", p.x, p.y);
                        d=loc.pos.x;
                        e=loc.pos.y;  
                        f=loc.pos.z;
                        double velocity;
                        velocity=fabs(sqrt(a*a+b*b+c*c)-sqrt(d*d+e*e+f*f));
                        printf("velocity%lf \n", velocity);
                        a=loc.pos.x;
                        b=loc.pos.y;
                        c=loc.pos.z;
                      
                }


         for (i = 0; i < loc.anchors.dist.cnt; ++i) {
                    printf("%u)", i);
                    printf("0x%04x", loc.anchors.dist.addr[i]);
                    printf("=%lu,%u \n", loc.anchors.dist.dist[i], loc.anchors.dist.qf[i]);}
                        
/*law of cosine*/
uint32_t dis[]={0,0,0};


for (i = 0; i < loc.anchors.dist.cnt; ++i) {
                    if (loc.anchors.dist.addr[i]==0xd4ae){
                            dis[0]=loc.anchors.dist.dist[i];}
                            //printf("loc.anchors.dist.sddr: %lu\n",loc.anchors.dist.dist[i] );
                            //printf("dis[0]: %lu\n",dis[0] );}
                    if (loc.anchors.dist.addr[i]==0x4aa4){
                            dis[1]=loc.anchors.dist.dist[i];}
                    if (loc.anchors.dist.addr[i]==0x9294){
                            dis[2]=loc.anchors.dist.dist[i];}}
double x,y,z,a_p,b_p,c_p,cos1,cos2,term1, term2;
        term1=(dis[0]*dis[0]-dis[1]*dis[1]+1200*1200)/2400;
        term2=(dis[0]*dis[0]-dis[2]*dis[2]+1200*1200)/2400;
        z=sqrt(dis[0]*dis[0]-term1*term1-term2*term2);
        printf("z axies is : %lf",z);
        z=0;
        a_p=sqrt(dis[0]*dis[0]-z);
        b_p=sqrt(dis[1]*dis[1]-z);
        c_p=sqrt(dis[2]*dis[2]-z);
        cos1=(1200*1200+a_p*a_p-c_p*c_p)/(2*1200*a_p);
        //printf("cos1:%f\n",cos1);
        y=cos1*a_p;
        cos2=(1200*1200+a_p*a_p-b_p*b_p)/(2*1200*a_p);
        //printf("cos2: %f\n",cos2);
        x=cos2*a_p;
        printf("Calculated position is : [%lf, %lf, %lf]\n\n", x, y, z);







    /*x=(dis[0]*dis[0]-dis[1]*dis[1]+pos[0]*pos[0])/(2*pos[0]);
    y=(dis[0]*dis[0]-dis[2]*dis[2]+pos[1]*pos[1])/(2*pos[1]);
    z=sqrt(dis[0]*dis[0]-x*x-y*y);
    printf("Calculated position is : [%ld, %ld, %ld]\n", x, y, z);*/



                printf("\n");
        }else {
                printf("err code: %d\n", rv);
        }  
            

        for (int i=100;i<100;i++)
        {i2cbyteXH = 0x28;
        rv = dwm_i2c_write(0x33 >> 1, &i2cbyteXH, 1, true);
	rv = dwm_i2c_read(0x33 >> 1, &i2cbyteXH, 1);
        i2cbyteXH = 0x29;
        rv = dwm_i2c_write(0x33 >> 1, &i2cbyteXH, 1, true);
	rv = dwm_i2c_read(0x33 >> 1, &i2cbyteXH, 1);
        i2cbyteX=(i2cbyteXH<<8)|i2cbyteXL;
        x=i2cbyteX/64*0.004;

        i2cbyteYH = 0x2a;
        rv = dwm_i2c_write(0x33 >> 1, &i2cbyteYH, 1, true);
	rv = dwm_i2c_read(0x33 >> 1, &i2cbyteYH, 1);
        i2cbyteYH = 0x2b;
        rv = dwm_i2c_write(0x33 >> 1, &i2cbyteYH, 1, true);
	rv = dwm_i2c_read(0x33 >> 1, &i2cbyteYH, 1);
        i2cbyteY=(i2cbyteYH<<8)|i2cbyteYL;
        y=i2cbyteY/64*0.004;

        i2cbyteZH = 0x2c;
        rv = dwm_i2c_write(0x33 >> 1, &i2cbyteZH, 1, true);
	rv = dwm_i2c_read(0x33 >> 1, &i2cbyteZH, 1);
        i2cbyteZH = 0x2d;
        rv = dwm_i2c_write(0x33 >> 1, &i2cbyteZH, 1, true);
	rv = dwm_i2c_read(0x33 >> 1, &i2cbyteZH, 1);
        i2cbyteZ=(i2cbyteZH<<8)|i2cbyteZL;
        z=i2cbyteZ/64*0.004;

        printf("acceleration data:x= %f, y=%f, z=%f\n",x, y, z);}
        
       

		
                /* Thread loop */
		rv = dwm_evt_wait(&evt);

		if (rv != DWM_OK) {
			printf("dwm_evt_wait, error %d\n", rv);
		} else {
			on_dwm_evt(&evt);
		}
	}
}

/**
 * Application entry point. Initialize application thread.
 *
 * @warning ONLY ENABLING OF LOCATION ENGINE OR BLE AND CREATION AND STARTING OF
 * USER THREADS CAN BE DONE IN THIS FUNCTION
 */
void dwm_user_start(void)
{
	uint8_t hndl;
	int rv;

	dwm_shell_compile();
	//Disabling ble by default as softdevice prevents debugging with breakpoints (due to priority)
	//dwm_ble_compile();
	dwm_le_compile();
	dwm_serial_spi_compile();

	/* Create thread */
	rv = dwm_thread_create(THREAD_APP_PRIO, app_thread_entry, (void*)NULL,
			"app", THREAD_APP_STACK_SIZE, &hndl);
	APP_ERR_CHECK(rv);

	/* Start the thread */
	dwm_thread_resume(hndl);
}