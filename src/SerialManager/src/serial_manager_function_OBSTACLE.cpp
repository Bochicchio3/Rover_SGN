//=================================================================================================
//
//   nodo ROS per leggere e scrivere su seriale
//
//=================================================================================================

#include "serial_manager.h"
#include "obstacle_avoidance/distance_msg.h"



/*****************************************************************/
/*                                                               */
/*                 PARSING DEL PACKET                            */
/*****************************************************************/
//funzione che riceve e decodifica i pacchetti in arrivo da PC
void parser_mess(unsigned char buffer){


        //DEBUG 
        //ROS_INFO_STREAM("Init  parser_mess ( called by read_from_serial "  );
        //ROS_INFO_STREAM("[parser_mess] new_packet:  " << new_packet  );
        // ROS_INFO_STREAM("Char arrived:  " << std::hex <<  buffer   );
        //ROS_INFO_STREAM("Char arrived:  " << std::dec <<  buffer   );

    //implementazione della macchina a stati
    switch(state_msg){
        case HEADER_1:

            if(buffer == HEADER_A)
            {
                state_msg=HEADER_2;
            }else
            {
                state_msg=HEADER_1;
            }
            break;

        case HEADER_2:
            if(buffer == HEADER_B)
            {
                state_msg=ID;
                //è stato riconosciuto un header-->è in arrivo un nuovo pacchetto
                
            }
            else
            {
                state_msg=HEADER_1;
            }
            break;

        case ID:
            //if(buffer == PAYLOAD_POSE_R)
            if(buffer == PAYLOAD_POSE)
            {
                state_msg=wait_PAYLOAD;
                //è stato riconosciuto un ID-->è in arrivo un nuovo pacchetto
                //ma non è ancora stato ricevuto e decodificato tutto
            }
            else
            {
                state_msg=HEADER_1;
            }
            break;




            /*********************************************************/
            //PACCHETTO CONTENTENTE POSIZIONI E ORIENTAZIONI
        case wait_PAYLOAD:
            if(offset<N_BYTE)
                {
                    coda_recv_seriale.push(buffer);
                    state_msg=wait_PAYLOAD;
                    offset++;
                    //ROS_INFO_STREAM("[parser_mess] offset:  " << offset  );   
                    
                }
            if(offset==N_BYTE)
                {
					 //ROS_INFO_STREAM("Arrived six numbers encoded ( and new_packet incremented  "  );   
                    new_packet++;
                    offset=0;
                    state_msg=HEADER_1;
                    //cout << "ricevuto payload " << endl;
                    //ROS_INFO_STREAM("[parser_mess] Good: we have received  payload "  );   
                    //ROS_INFO_STREAM("[parser_mess] new_packet:  " << new_packet  );   
                }

            break;
    }

    return;
}



/*****************************************************************/
/*                                                               */
/*                 DECODE PACKET                                 */
/*****************************************************************/
void decode_packet()
{
   //ROS_INFO_STREAM("FILL ARRAY ARRIVED FROM STM32F4   "  ); 
    for(int i=0;i<N_NUMBERS;i++){
                  new_grafic_stm[i] = decode_payload();
          //ROS_INFO(" %d  ",i  );         
    }
    new_packet--; 
    
}

/*****************************************************************/
/*                                                               */
/*                 DECODE PAYLOAD                                */
/*****************************************************************/
//By Ale
double decode_payload()
{
	  union D{
		   char   s[4];
		   float  n;
	     } d;
	         
    //in coda_recv_seriale ho 4 bytes da decodificare
    d.s[0]= coda_recv_seriale.front();//ROS_INFO("d.s[0]:%0X  ",d.s[0] );   
    coda_recv_seriale.pop();
    d.s[1] = coda_recv_seriale.front();//ROS_INFO("d.s[1]:%0X ",d.s[1] );
    coda_recv_seriale.pop();
    d.s[2] = coda_recv_seriale.front() ;//ROS_INFO("d.s[2]:%0X " , d.s[2] );
    coda_recv_seriale.pop();
    d.s[3] = coda_recv_seriale.front();//ROS_INFO("d.s[3]:%0X ", d.s[3] );
    coda_recv_seriale.pop();
  
    //ROS_INFO(" %f ", d.n );
    
    return (double) d.n;
}

/*****************************************************************/
/*                                                               */
/*                  ENCODE PAYALOD                               */
/*****************************************************************/
//By Ale 
void encode_payload(double payload)
{

   		union D{
		   char   s[4];
		   float  n;
	     } d;
	         
     d.n = (float) payload;
    
    coda_send_seriale.push((unsigned char) d.s[0] ); 
    coda_send_seriale.push((unsigned char) d.s[1] );
    coda_send_seriale.push((unsigned char) d.s[2] );
    coda_send_seriale.push((unsigned char) d.s[3] );
 
}

/*****************************************************************/
/*                                                               */
/*                 WRITE SERIALE                                 */
/*****************************************************************/
void write_to_serial(int* serial)
{
    while( !coda_send_seriale.empty() )
    {
        write(*serial,&coda_send_seriale.front(), 1);
        coda_send_seriale.pop();
    }
}

/*****************************************************************/
/*                                                               */
/*                 READ SERIALE                                  */
/*****************************************************************/
void read_from_serial(int* serial)
{

    static int recv1 = 0;
    static int recv2 = 0;
    int bytes = 0;
    int sum_bytes = 0;
    unsigned char buf[1024];

    //DEBUG 
    //ROS_INFO_STREAM("Init  read_from_serial "  );
    //ROS_INFO_STREAM("[read_from_serial] new_packet:  " << new_packet  );
    // Read data from the COM-port
    bytes= read(*serial, buf, sizeof buf);

    for(int a = 0 ; a < bytes ; a++)
    {
        parser_mess(buf[a]);
        
    }

    while(new_packet)
    {
       
         decode_packet();
         
         ROS_INFO(" ----------------------------\n "  );  
	     for(int i=0;i<N_NUMBERS;i++)
              ROS_INFO("[%d]=%f  ",i, new_grafic_stm[i]);
          
         //ROS_INFO_STREAM("PUBLISHING  A POSE MESSAGE   "  );  
      }



}
/*****************************************************************/
/*                                                               */
/*                 INIT SERIALE                                  */
/*****************************************************************/
int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
        printf("error %d from tcgetattr", errno) ;
        //error_message("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf("error %d setting term attributes", errno);
}


int serial_init(int* fd,const char* seriale_dev)
{

    /* apro la porta seriale*/
    const char * portname = seriale_dev;

     *fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (*fd < 0)
    {
        printf("error %d opening %s: %s", errno, portname, strerror (errno));
        return -1;
    }
    /*imposto baud rate*/
    set_interface_attribs (*fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (*fd, 0);

    /*inizializzo variabili macchina a stati*/
    state_msg = HEADER_1;
    new_packet = 0;

 
    return 1;
}


bool isDifferent(double* upboard , double* stm,double* x, double* y,int* pos){
	
	bool result = false;
	double a,b,eps;
	
	eps=0.3;
	
	for(int i=0; i< N_NUMBERS; i++){
          //*(upboard+i)    *(stm+i)
          a=*(upboard+i);
          b=*(stm+i);
      
          if( abs(a-b) > eps ){
		  *pos=i;
		  *x=a;
		  *y=b;  	  
		  result = true;
		  break;       
	      }	
	}
	
	return result;
}

