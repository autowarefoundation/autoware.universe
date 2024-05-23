extern "C"
{
#include "node_api.h"   
#include "operator_api.h"
#include "operator_types.h"
}

#include <iostream>
#include <vector>

bool to_exit_process;

int run(void *dora_context)
{
    unsigned char counter[3] ;

    //for (int i = 0; i < 20; i++)
    to_exit_process = false;
    counter[0] = 0; 
    while(!to_exit_process)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        { 
            char* output_data = (char *)counter;
             
            counter[0] += 1;
            if(counter[0]>=255)
            	counter[0]=0;

            std::string out_id = "counter_A";
            size_t data_len = 3 ;
            int resultend=dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, data_len);
           
           std::cout
                << "dora_send_output: out_id "<<out_id<< "  out_data_len: "<<data_len<<std::endl;
                
            if (resultend != 0)
            {
                std::cerr << "failed to send output" << std::endl;
                return 1;
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }
    return 0;
}

int main()
{
    std::cout << "dora node A driver" << std::endl;

    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);
   
 
    to_exit_process = true;
 
    std::cout << "exit dora node A ..." << std::endl;
    return ret;
}

