extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}


#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <string>

#include <nlohmann/json.hpp>
using json = nlohmann::json;


int run(void *dora_context)
{

    while (true)
    {

        void * event = dora_next_event(dora_context);

        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            int len = data_len;
            
            std::cout << "=======================" <<std::endl;
            std::cout << "Input Data length: " << data_len <<"   " <<data_id<<std::endl;
            if (strcmp("counter_A", data_id) == 0)
            {
		char* data_tem_a = reinterpret_cast<char*>(data); 
		int num_points = data_len / sizeof(char); 
		std::vector<float> data_node_a(data_tem_a, data_tem_a + num_points);
               std::cout << " counter_A Input Data: " <<data_node_a[0]<< std::endl;
            }
            else if (strcmp("counter_B", data_id) == 0)
            {
            	char* data_tem_b = reinterpret_cast<char*>(data); 
		int num_points = data_len / sizeof(char); 
		std::vector<float> data_node_b(data_tem_b, data_tem_b + num_points);
               std::cout << "counter_B Input Data: " <<data_node_b[0]<< std::endl;
            }
            std::cout << "=======================" <<std::endl;
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
    std::cout << "dora node c" << std::endl;

    auto dora_context = init_dora_context_from_env();

    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "END dora node c" << std::endl;

    return ret;
}


