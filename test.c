#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>

char Dev[] = "/dev/sw0" ;
extern int errno ;

unsigned short int BufToShort( unsigned char *Buf)
{
  return ((unsigned short int)Buf[ 1] | (unsigned short int)Buf[ 0] << 8) ;
} ;


int main( int ArgC, char *ArgV[])
{
  unsigned char Buf[ 10] ;

  int fd ;
  fd = open( Dev, O_RDONLY) ;
  if ( fd < 0) { printf("error open device, errno=%i %s\n", errno, strerror(errno) ) ; return 1 ; } ;
  if ( read( fd, Buf, 5) == -1) { printf("error read, errno=%i %s\n", errno, strerror(errno) ) ; return 1 ; } ;
  close( fd) ;
//  int i = 0 ; while ( i < 5)  { printf("%x ", Buf[ i]) ; i = i + 1 ; } ;
//  printf("\n") ;
  int Hum = BufToShort( &Buf[ 0]) ;  
  int Temp = BufToShort( &Buf[ 2]) ;
  if ( Temp & 0x8000) Temp = (Temp & 0x7FFF)*-1 ;
  printf("h:%d.%d %%, t:%d.%d C\n",Buf[0],Buf[1],Buf[2],Buf[3]);
  printf("Hum=%i\nTem=%i\n", Hum, Temp) ;
  return 0 ;
} ;
