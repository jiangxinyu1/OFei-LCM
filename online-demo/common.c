/*************************************************************************
> File Name: common.c
> Author:
> Mail:
> Created Time:
************************************************************************/

#include "common.h"
#include "DepthMapWrapper.h"
//2020-3-21
void writeDepthData(char* filename, float *depth_data, unsigned int numPoints)
{
        FILE *fp = NULL;
	unsigned int i;
	unsigned int times = 0;
	float agv=0.0f;

        depth_data_pkt_t *depth__data = ((depth_data_pkt_t *)depth_data);

    fp = fopen(filename, "w"); //save more frames
    if(!fp)
    {
        printf(" Open %s error\n", (char *)filename);

        return;
    }
        fprintf(fp,"%s\n", "ply");
        fprintf(fp,"%s\n", "format ascii 1.0");
        fprintf(fp,"%s %d\n", "element vertex", numPoints);
        fprintf(fp,"%s\n", "property float32 x");
        fprintf(fp,"%s\n", "property float32 y");
        fprintf(fp,"%s\n", "property float32 z");
        fprintf(fp,"%s\n", "end_header");
//      for (unsigned int i = 0; i < numPoints * 4; i += 4)     //2020-3-21
        for (i = 0; i < numPoints; i ++)
    {       
            
	    if(depth__data[i].noise<= 0.01)
	    {
                fprintf(fp,"%f %f %f\n", depth__data[i].X, depth__data[i].Y, depth__data[i].Z);
	    }
	    else
	    {
                fprintf(fp,"%f %f %f\n", 0.0, 0.0, 0.0);
	    }
            
            //fprintf(fp,"%f %f %f %f\n", depth__data[i].X, depth__data[i].Y, depth__data[i].Z, depth__data[i].noise);
        }

	
	fclose(fp);
}


void writePointCloud(char* filename, float *pointCloud, unsigned int numPoints)
{
	FILE *fp = NULL;
	
	unsigned int i;

	pc_pkt_t *point_Cloud = ((pc_pkt_t *)pointCloud);

    fp = fopen(filename, "w"); //save more frames
    if(!fp)
    {
        printf(" Open %s error\n", (char *)filename);

        return;
    }
	fprintf(fp,"%s\n", "ply");
	fprintf(fp,"%s\n", "format ascii 1.0");
	fprintf(fp,"%s %d\n", "element vertex", numPoints);
	fprintf(fp,"%s\n", "property float32 x");
	fprintf(fp,"%s\n", "property float32 y");
	fprintf(fp,"%s\n", "property float32 z");
	fprintf(fp,"%s\n", "end_header");

	for (i = 0; i < numPoints; i ++)	//2020-3-22
    {
//		fprintf(fp,"%f %f %f\n", pointCloud[i], pointCloud[i+1], pointCloud[i+2]);//2020-3-21
		fprintf(fp,"%f %f %f %f\n", point_Cloud[i].X, point_Cloud[i].Y, point_Cloud[i].Z,point_Cloud[i].No);
	}
	//printf("\n\n writePointCloud:: numPoints is %d , i is %d . \n\n", numPoints, i);
	fclose(fp);
}

int save_frame_to_file(void *str,void *start,int length)
{
    FILE *fp = NULL;

    fp = fopen(str, "wb+"); //save more frames
    if(!fp)
    {
        printf(" Open %s error\n", (char *)str);

        return -1;
    }

    if(fwrite(start, length, 1, fp))
    {
        fclose(fp);

        return 0;
    }
    else
    {
        printf(" Write file fail (%s)\n",strerror(errno));
        fclose(fp);

        return -1;
    }

    return 0;
}
