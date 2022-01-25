
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>


/**************** 0366 ********************/
//#define IGF_SIZE  86806 
//#define JGF_START 0x5F00  //24320
//#define JGF_START 0x5EFC


#define IGF_OFFSET (4)
#define BYTES_PER_PAGE       256          /* one eeprom page is 256 byte */
#define DEVICE_ADDR      0x3D

#define ZWE_OFFSIZE 0x2000
#define CAL_OFFSIZE 53

unsigned int igf_size = 0;

/* write len bytes (stored in buf) to eeprom at address addr, page-offset offset */
/* if len=0 (buf may be NULL in this case) you can reposition the eeprom's read-pointer */
/* return 0 on success, -1 on failure */
int i2c_write_reg(int fd, unsigned int addr, unsigned short reg, unsigned short val)
{
	struct i2c_rdwr_ioctl_data msg_rdwr;
	struct i2c_msg             i2cmsg;
	int i;
	unsigned char buf[4];

	buf[0] = reg >> 8;	
	buf[1] = reg & 0xff;
	
	buf[2] = val>>8;	
	buf[3] = val & 0xff;
//	fprintf(stderr,"i2c_write_reg  reg 0x%04x  ::  0x%04x\n", reg, val);	
	msg_rdwr.msgs = &i2cmsg;
	msg_rdwr.nmsgs = 1;
	i2cmsg.addr  = addr;
	i2cmsg.flags = 0;
	i2cmsg.len   = 4;
	i2cmsg.buf   = buf;

	if((i=ioctl(fd,I2C_RDWR,&msg_rdwr))<0){
	    perror("i2c_write_reg ioctl() \n");
	    fprintf(stderr,"ioctl returned %d  reg %02x\n",i, reg);
	    return -1;
	}

	return 0;
}

unsigned short i2c_read_reg(int fd, unsigned int addr, unsigned short reg)
{
	struct i2c_rdwr_ioctl_data msg_rdwr;
	struct i2c_msg             msg[2];
	int i;
	unsigned char buf[2];
	unsigned char val[2];
	//unsigned short val;

	buf[0] = reg >> 8;	
	buf[1] = reg & 0xff;

	msg[0].addr = addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;
	
	
	msg[1].addr = addr;
	msg[1].flags = 1;
	msg[1].len = 2;
	msg[1].buf = val;

	msg_rdwr.msgs = msg;	
	msg_rdwr.nmsgs = 2;


	if((i=ioctl(fd,I2C_RDWR,&msg_rdwr))<0){
	    perror("i2c_read_reg ioctl() \n");
	    fprintf(stderr,"ioctl returned %d  reg 0x%04x\n",i, reg);
	    return -1;
	}

	//fprintf(stderr,"SPI_status:0x%04x :  0x%04x\n", reg, val);
	return val[0]<<8|val[1];
}

int i2c_block_read(int file,int dev_addr,int eeprom_addr,unsigned char *buf, int len){

	struct i2c_rdwr_ioctl_data msg_rdwr;
	struct i2c_msg             msg[2];
	int ret;
	unsigned char reg[2];
	reg[0] = eeprom_addr >> 8;	
	reg[1] = eeprom_addr & 0xff;
	
	msg[0].addr = dev_addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = reg;
	
	msg[1].addr = dev_addr;
	msg[1].flags = 1;
	msg[1].len = len;
	msg[1].buf = buf;

	msg_rdwr.msgs = msg;	
	msg_rdwr.nmsgs = 2;
	
	if ((ret = ioctl(file, I2C_RDWR, &msg_rdwr)) < 0) {
		fprintf(stderr, "Error: Read error:%d  %02x%02x\n", ret, reg[0], reg[1]);
	   	return ret;                     
	}

	//fprintf(stderr, "i2c_block_read: %d  reg %02x%02x len %d  val 0x%04x \n", ret, reg[0],reg[1],len, buf[0]);
        
        //for(int i=0; i<len; i++)
        //  fprintf(stderr, "i2c_block_read: val 0x%04x \n", buf[i]);
	
	return ret;
}

int eeprom_open(char* dev_name, unsigned int addr)
{
   int fd;
   unsigned short read_value = 0;

   if((fd=open(dev_name,O_RDWR))<0){
      fprintf(stderr,"Could not open i2c at %s\n",dev_name);
      perror(dev_name);
      exit(1);
   }

   i2c_write_reg(fd, addr,0xA087, 0xC001);	 // enable SPI block
   //i2c_write_reg(fd, addr,0xA087, 0x5003);
   read_value = i2c_read_reg(fd, addr,0xA087);
   fprintf(stderr,"readaddress 0xA087  0x%04x\n" ,read_value);
   i2c_write_reg(fd, addr,0xA088, 0x0100);       // block to write to SPI flash
   i2c_write_reg(fd, addr,0xA089, 0x0000);       // block read out of SPI flash

   return fd;
}

void get_module_serial(int fd, unsigned int addr,unsigned short snreg, unsigned short *sn)
{
	//unsigned short sn[4];

	for(int i = 0; i< 4; i++){
		sn[i] = i2c_read_reg(fd, addr,snreg+i);
	}
	fprintf(stderr,"module SN  %04x-%04x-%04x-%04x \n",sn[0], sn[1], sn[2], sn[3]);
}

/* read len bytes stored in eeprom at address addr, offset offset in array buf */
/* return -1 on error, 0 on success */
int eeprom_read(int fd,
		 unsigned int addr,
		 unsigned int eeprom_addr,
		 unsigned char *buf,
		 unsigned int len
){
 	int tt = 0;
	unsigned short read_value = 0;
	unsigned short SPI_status = 0;
	unsigned short SPIread_address = 0x0000;
	unsigned short SPIwriteaddress = 0x0100;
	
//	unsigned short write_command = 0x0300 | (((eeprom_addr) & 0xFF0000) >> 16);
//	unsigned short	lower_address = (eeprom_addr) & 0x00FFFF;
//	i2c_write_reg(fd, addr,SPIwriteaddress, write_command);       // read SPI / upper Address bits
//	i2c_write_reg(fd, addr,SPIwriteaddress + 1, lower_address);        // lower address bits
	
//	//fprintf(stderr,"eeprom_addr 0x%08x page 0x%04x command  0x%04x  0x%04x\n",
//	//eeprom_addr, (eeprom_addr-JGF_START)/256,write_command, lower_address);
														
													
//	unsigned short num_read = len + 3 + 0xE000;
//	i2c_write_reg(fd, addr,0xA08A, num_read);       // enable read SDI
//	i2c_write_reg(fd, addr,0xA08B, 0x0002);        // init and execute
	
//	SPI_status = i2c_read_reg(fd, addr,0xA08C);	
	
		if(tt == 1)
	{
		tt=1;
		fprintf(stderr,"eeprom_addr  0x%04x\n" ,eeprom_addr);
	}
	read_value = i2c_read_reg(fd, addr,0x9403);
	//fprintf(stderr,"readaddress 0x9403  0x%04x\n" ,read_value);
	read_value = i2c_read_reg(fd, addr,0x8423);
	//fprintf(stderr,"readaddress 0x8423  0x%04x\n" ,read_value);
	read_value = i2c_read_reg(fd, addr,0x9403);
	//fprintf(stderr,"readaddress 0x9403  0x%04x\n" ,read_value);

	unsigned short write_command = 0x0300 | (((eeprom_addr) & 0xFF0000) >> 16);
	//fprintf(stderr,"wrte_command 0x%04x \n",write_command);
	unsigned short	lower_address = (eeprom_addr) & 0x00FFFF;
	//fprintf(stderr,"low_address 0x%04x\n",lower_address);

	i2c_write_reg(fd, addr,SPIwriteaddress, write_command);       // read SPI / upper Address bits
	read_value = i2c_read_reg(fd, addr,SPIwriteaddress);
	//fprintf(stderr,"readaddress 0x%04x  read_value = 0x%04x\n",SPIwriteaddress ,read_value);
	i2c_write_reg(fd, addr,SPIwriteaddress + 1, lower_address);        // lower address bits
	read_value = i2c_read_reg(fd, addr,SPIwriteaddress + 1);
	//fprintf(stderr,"readaddress 0x%04x  0x%04x\n",SPIwriteaddress+1 ,read_value);

	//fprintf(stderr,"eeprom_addr 0x%08x page 0x%04x command  0x%04x  0x%04x\n",
	//eeprom_addr, (eeprom_addr-JGF_START)/256,write_command, lower_address);

//	i2c_write_reg(fd, addr,0xA087, 0xC001);				     // enable SPI block
	read_value = i2c_read_reg(fd, addr,0xA087);
	//fprintf(stderr,"readaddress 0xA087  0x%04x\n" ,read_value);
//	i2c_write_reg(fd, addr,0xA088, 0x0100);		// block to write to SPI flash
	read_value = i2c_read_reg(fd, addr,0xA088);
	//fprintf(stderr,"readaddress 0xA088  0x%04x\n" ,read_value);
//	i2c_write_reg(fd, addr,0xA089, 0x0000);       // block read out of SPI flash
	read_value = i2c_read_reg(fd, addr,0xA089);
	//fprintf(stderr,"readaddress 0xA089  0x%04x\n" ,read_value);
	unsigned short num_read =len + 3 + 0xE000;// len + 3 + 0xE000;
	i2c_write_reg(fd, addr,0xA08A, num_read);       //// enable read SDI
	read_value = i2c_read_reg(fd, addr,0xA08A);
	//fprintf(stderr,"readaddress 0xA08A  0x%04x\n" ,read_value);
	i2c_write_reg(fd, addr,0xA08B, 0x0002);        // init and execute
	read_value = i2c_read_reg(fd, addr,0xA08B);
	//fprintf(stderr,"readaddress 0xA08B  0x%04x\n" ,read_value);

	SPI_status = i2c_read_reg(fd, addr,0xA08C);
	if(SPI_status!= 0x0001)
	{
		fprintf(stderr,"SPI_status:0xA08C eeprom_addr 0x%08x  0x%04x\n",eeprom_addr ,SPI_status);
		usleep(1);
	}
	
	i2c_block_read(fd, addr, SPIread_address, buf, len);
/*
        int j=0;
        for(int i=0; i<len/2; i++)
        {
           read_value = i2c_read_reg(fd, addr,SPIread_address+i);
           buf[j++] = read_value >> 8;
           buf[j++] = read_value;
        }
*/
	return SPI_status;
}

bool need_updata_calibration(char *fn, unsigned short *sn)
{
	int f=-1;
	int len = 0;	
	//int callen = IGF_SIZE +4*2;
        int callen = igf_size +4*2;
	char caldata[callen];
	char *header = "PMDJGF"; // "PMDJGF"
	f=open(fn,O_RDONLY,0644);
	if(f >= 0)
	{		
		len = read(f, caldata, callen);				
		
		if((len == callen) && (!memcmp(&caldata[0], header, 6)) && (!memcmp(&caldata[len-8], sn, 8)))
		{
			close(f);
			return false;
		} 
		else
		{
			close(f);			
			return true;
		}
	}
	else{
		printf("updata calbration data\n");
		return true;
	}
	
}


int get_calibration_data(char* bus_name, char* fn)
{  
   int i = 0;
   unsigned short sn[4];

   /* filedescriptor and name of device */
   int fd = 0;
   /* filedescriptor and name of data file */
   int fd1 = 0; 

   unsigned int addr = DEVICE_ADDR;
   int read_size = 0;
   unsigned int jgfaddr = 0;
   unsigned char cal_msg[6];

   fd = eeprom_open(bus_name, addr);  
   get_module_serial(fd, addr, 0xA098, sn);

   eeprom_read(fd,addr,ZWE_OFFSIZE+CAL_OFFSIZE,cal_msg,6);
   //printf("cal_msg = %x %x %x %x %x %x \n", cal_msg[0],cal_msg[1],cal_msg[2],cal_msg[3],cal_msg[4],cal_msg[5]);

   jgfaddr = cal_msg[0]<<8 | cal_msg[1];
   printf("jgfaddr = %x\n", jgfaddr);
   jgfaddr -= 4;

   igf_size = cal_msg[4]<<16 | cal_msg[3]<<8 | cal_msg[2];
   printf("igf_size = %d\n", igf_size);
   read_size = igf_size + IGF_OFFSET;

   int pages;
   int bytesfirst;
   int byteslast;
   unsigned char buf[read_size];
   unsigned char* pb = buf;

   if(need_updata_calibration(fn,sn) == false)
   {
      close(fd);
      return 0;
   }
  
   //i2c_write_reg(fd, addr,0x9400, 0x0000);  
//sleep(1);

   fd1 = open(fn,O_WRONLY|O_CREAT,0666);
   if(fd1 < 0){
      fprintf(stderr,"Could not open pmd.spc. \n");
      exit(1);
   }

   if(jgfaddr % BYTES_PER_PAGE)	{
      bytesfirst = BYTES_PER_PAGE - jgfaddr % BYTES_PER_PAGE;
   } else {
      bytesfirst = 0;
   }	
   pages = (read_size - bytesfirst)/BYTES_PER_PAGE;

   byteslast = read_size - bytesfirst - pages*BYTES_PER_PAGE;	

   //jgfaddr = JGF_START;
   pb = buf;
   if(bytesfirst)
   {
      eeprom_read(fd,addr,jgfaddr,pb,bytesfirst);
   }

   jgfaddr += bytesfirst;
   pb += bytesfirst;

   for(i= 0; i<pages; i++)
   {
      eeprom_read(fd,addr,jgfaddr+i*BYTES_PER_PAGE,pb+i*BYTES_PER_PAGE,BYTES_PER_PAGE);
      printf("read page = %d\n", i);
   }

   jgfaddr += i*BYTES_PER_PAGE;	
   pb += i*BYTES_PER_PAGE;	
	
   if(byteslast)
   {
      eeprom_read(fd,addr,jgfaddr,pb,byteslast);
   }

   write(fd1, buf+IGF_OFFSET, read_size-IGF_OFFSET);
   write(fd1, sn, 4*2);

//i2c_write_reg(fd, addr,0x9400, 0x0001);   
//sleep(1);

   close(fd);
   close(fd1);

   return 0;
}                    








