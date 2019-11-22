#include <stdio.h>
#include <string.h>
#include <stdint.h>

/*
	To compile for debugging:
		gcc circ_buff.c -o circ_buff && ./circ_buff && rm circ_buff
		This would: - Compile the c file 		(gcc circ_buff.c -o circ_buff)
					- Run the compiled file 	(./circ_buff)
					- Delete the compiled file 	(rm circ_buff)
	To compile for running:
		cc -fPIC -shared -o circ_buff.so circ_buff.c
*/

/* 
	This function changes the value of a binary array passed in by reference. 
	If the put idx passes the next frame boundary, the pop_array puts a 1 in the correct 
	location and updates the get idx to be frame boundary.
*/

short find_pops(long long  old_put_idx, 
			   long long  new_put_idx, 
			   long long  frame_size)
{
	int old_frame_idx = old_put_idx / frame_size;
	int new_frame_idx = new_put_idx / frame_size;
	return(old_frame_idx == new_frame_idx) ? -1 : old_frame_idx;
}

void add_zeros(long long num_zeros, 
			   short* buffer,
			   long long buffer_len, 
			   long long* put_idx,
			   long long frame_size, 
			   short* pop_frame_idx) 
{
	int new_put_idx = *put_idx;
	int to_end_of_frame = frame_size - (*put_idx % frame_size); // number of cell to the end of current frame
	if (num_zeros < to_end_of_frame) { // if does not reach the end of the frame
		memset(buffer + *put_idx, 0, num_zeros * sizeof(buffer[0])); // set zeros accordingly
		new_put_idx += num_zeros; // move pointer
	}
	else {	// if overflow
		memset(buffer + *put_idx, 0, to_end_of_frame * sizeof(buffer[0])); // set all unfinished cell of frame to 0
		num_zeros -= to_end_of_frame; // substract those 0 filled
		new_put_idx += to_end_of_frame; // move to start of the next frame
		if (new_put_idx >= buffer_len) new_put_idx -= buffer_len; // loop if necessary

		num_zeros %= frame_size; // the rest of the needed zeros (skip 0-filled frames)
		//printf("if %lli \n", num_zeros);
		memset(buffer + new_put_idx, 0, num_zeros * sizeof(buffer[0])); // fill zeros to destination idx
		new_put_idx += num_zeros; // move pointer
	}
	*pop_frame_idx = find_pops(*put_idx, new_put_idx, frame_size);
	*put_idx = new_put_idx;
	//printf("[C] Current put: %lld | Pop index: %d \n", *put_idx, *pop_frame_idx);
}

void add_msg(short* msg,
			 short msg_len, 
			 short* buffer,
			 long long buffer_len, 
			 long long* put_idx,
			 long long frame_size, 
			 short* pop_frame_idx) 
{
	int new_put_idx = *put_idx;
	if (*put_idx + msg_len <= buffer_len) //did not loop around to beginning
	{
		memcpy(buffer + *put_idx, msg, msg_len*sizeof(msg[0]));
		new_put_idx = *put_idx + msg_len; //new location of put idx
		if (new_put_idx >= buffer_len) new_put_idx -= buffer_len;
	}
	else // did loop around to beginning
	{	
		memcpy(buffer + *put_idx, msg, (buffer_len - *put_idx)*sizeof(msg[0]));
		memcpy(buffer, msg + (buffer_len - *put_idx), (msg_len - buffer_len + *put_idx)*sizeof(msg[0]));
		new_put_idx = msg_len - buffer_len + *put_idx;
	}

	*pop_frame_idx = find_pops(*put_idx, new_put_idx, frame_size);
	*put_idx = new_put_idx;
	//printf("[C] Current put: %lld | Pop index: %d \n", *put_idx, *pop_frame_idx);
}

// int main()
// {
//     printf("\n\n\nHello World\n");
    
// 	int buffer[15] = {0};
//     int pop_array[5] = {0};
//     int buffer_len = 15;
//     int frame_size = 3;
//     long long int put_idx =1;
//     long long int get_idx = 0;
//     int msg[5] = {1,2,3,4,5};    

//     // indices
//     for ( int ii = 0; ii<buffer_len;ii++){
//         printf("%i ", ii);
//     }
//     printf("\n\n");
//     // before modification
//     for ( int ii = 0; ii<buffer_len;ii++){
//         printf("%i ", buffer[ii]);
//     }
    
//     add_msg(msg,5,buffer,buffer_len,&put_idx,&get_idx,frame_size,pop_array);
//     printf("\n");
//     for ( int ii = 0; ii<buffer_len;ii++){
//         printf("%i ", buffer[ii]);
//     }
//     printf("\n");

//     for ( int ii = 0; ii<5;ii++){
//         printf("%i ", pop_array[ii]);
//     }
//     add_msg(msg,5,buffer,buffer_len,&put_idx,&get_idx,frame_size,pop_array);
//     printf("\n");
//     for ( int ii = 0; ii<buffer_len;ii++){
//         printf("%i ", buffer[ii]);
//     }
//     printf("\n");

//     for ( int ii = 0; ii<5;ii++){
//         printf("%i ", pop_array[ii]);
//     }
//     add_msg(msg,5,buffer,buffer_len,&put_idx,&get_idx,frame_size,pop_array);
//     printf("\n");
//     for ( int ii = 0; ii<buffer_len;ii++){
//         printf("%i ", buffer[ii]);
//     }
//     printf("\n");

//     for ( int ii = 0; ii<5;ii++){
//         printf("%i ", pop_array[ii]);
//     }
//     add_zeros(5,buffer,buffer_len,&put_idx,&get_idx,frame_size,pop_array);
//     printf("\n");
//     for ( int ii = 0; ii<buffer_len;ii++){
//         printf("%i ", buffer[ii]);
//     }
//     printf("\n");

//     for ( int ii = 0; ii<5;ii++){
//         printf("%i ", pop_array[ii]);
//     }

    
//         printf("\n");
//     // after modification
//     for ( int ii = 0; ii<buffer_len;ii++){
//         printf("%i ", buffer[ii]);
//     }
    
    
//     return 0;
// }