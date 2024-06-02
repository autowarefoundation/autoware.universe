// extern "C"
// {
// #include "node_api.h"
// #include "operator_api.h"
// #include "operator_types.h"
// }
extern "C"
{
#include "/home/crp/dora/apis/c/node/node_api.h"
#include "/home/crp/dora/apis/c/operator/operator_api.h"
#include "/home/crp/dora/apis/c/operator/operator_types.h"
}

#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <pqxx/pqxx>
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>
using json = nlohmann::json;
using namespace pqxx;
using namespace std;

#define ID_SUM 1

string dbname;
string user;
string password;
string hostaddr;
string port;
string address;
string angle;  

typedef struct LanePoint{
    double x;
    double y;
    int sortt;
    int road_id;
} LanePoint;

typedef struct Lane
{
    int current_lane_id;
    int next_lane_id;
    vector<LanePoint> LanePoints_right_current_line;
    vector<LanePoint> LanePoints_left_current_line;
    vector<LanePoint> ReferencePoints_current_line;
    vector<LanePoint> LanePoints_right_next_line;
    vector<LanePoint> LanePoints_left_next_line;
    vector<LanePoint> ReferencePoints_next_line;
    int turn_flag;
    int current_lane_num;
    int v;
    int scenes_id;
    LanePoint current_lane_start_point;
    LanePoint current_lane_end_point;
} Lane_Message;

Lane_Message lane;


void ConfigFileRead()
{
    ifstream configFile;
    const char* env_p = std::getenv("SEED_HOME");
    string path = env_p;   
    path += "/setting.conf";
    configFile.open(path.c_str());
    string strLine;
    if(configFile.is_open())
    {
        while (!configFile.eof())
        {
            getline(configFile, strLine);
            size_t pos = strLine.find('=');
            string key = strLine.substr(0, pos);

            if (key == "dbname") {
                dbname=strLine.substr(pos + 1);
            } else if(key == "user"){
                user = strLine.substr(pos + 1);
            } else if(key == "password") {
                password = strLine.substr(pos + 1);
            } else if(key == "hostaddr") {
                hostaddr = strLine.substr(pos + 1);
            } else if(key == "port") {
                port = strLine.substr(pos + 1);
            } else if(key == "angle"){
                angle = strLine.substr(pos + 1);
            } else if(key == "address"){
                address = strLine.substr(pos + 1);
            }

        }
    }
    else
    {
        cout << "Cannot open config file!" << endl;
    }
    
}


vector<LanePoint> get_referencr_point_zqy(int Lane_number){
    vector<LanePoint> points;
    LanePoint point;
	LanePoint point_after_tran;
    string sql;

    ConfigFileRead();
    string tablename = "reference_linkpoints_"+address;
    double angle_flout = atof(angle.c_str());
    try
    {
        connection C("dbname="+dbname+" user="+user+" password="+password+" hostaddr="+hostaddr+" port="+port);
        if (C.is_open())
        {
            cout << "Opened database successfully: " << C.dbname() << endl;
        }
        else
        {
            cout << "Can't open database" << endl;
        }
        sql = "SELECT pointorder,orig_fid,point_x,point_y from "+ tablename +" where orig_fid="+to_string(Lane_number) +" order by pointorder";
        work N1(C);
        result R1( N1.exec( sql ));
        N1.commit();
        for (result::const_iterator c = R1.begin(); c != R1.end(); ++c)
        {
            point.sortt = c[0].as<int>();
            point.road_id = c[1].as<int>();
            point.x = c[2].as<double>();
            point.y = c[3].as<double>();
            double new_x = cos(angle_flout * M_PI / 180) * point.x - sin(angle_flout * M_PI / 180) * point.y;
            double new_y = sin(angle_flout * M_PI / 180) * point.x + cos(angle_flout * M_PI / 180) * point.y;
            point.x = new_x;
            point.y = new_y;
            points.push_back(point);
        }

        cout << "Operation done successfully" << endl;
        C.disconnect ();
    }
    catch (const std::exception &e)
    {
	cerr<<e.what() << std::endl;
    }
    return points;
}


vector<LanePoint> get_lanelink_point_zqy(int Lane_number, string tablename){
    vector<LanePoint> points;
    LanePoint point;
	LanePoint point_after_tran;
    string sql;
    ConfigFileRead();
    double angle_flout = atof(angle.c_str());
    try
    {
        connection C("dbname="+dbname+" user="+user+" password="+password+" hostaddr="+hostaddr+" port="+port);
        if (C.is_open())
        {
            cout << "Opened database successfully: " << C.dbname() << endl;
        }
        else
        {
            cout << "Can't open database" << endl;
        }
        
        sql = "SELECT pointorder,orig_fid,point_x,point_y from "+ tablename +" where orig_fid ="+to_string(Lane_number)+" order by pointorder";
        work N1(C);
        result R1( N1.exec( sql ));
        N1.commit();
        for (result::const_iterator c = R1.begin(); c != R1.end(); ++c)
        {
            point.sortt = c[0].as<int>();
            point.road_id = c[1].as<int>();
            point.x = c[2].as<double>();
            point.y = c[3].as<double>();
            
            double new_x = cos(angle_flout * M_PI / 180) * point.x - sin(angle_flout * M_PI / 180) * point.y;
            double new_y = sin(angle_flout * M_PI / 180) * point.x + cos(angle_flout * M_PI / 180) * point.y;
            point.x = new_x;
            point.y = new_y;
            points.push_back(point);
        }
        
        cout << "Operation done successfully" << endl;
        C.disconnect ();
    }
    catch (const std::exception &e)
    {
	cerr<<e.what() << std::endl;
    }
    return points;
}


Lane_Message select_Lane_by_id(int id)
{
    string sql;
    string tablename;
    ConfigFileRead();

    try
    {
        tablename = "reference_link_" + address;
        connection C("dbname="+dbname+" user="+user+" password="+password+" hostaddr="+hostaddr+" port="+port);
        if (C.is_open())
        {
            cout << "Opened database successfully: " << C.dbname() << endl;
        }
        else
        {
            cout << "Can't open database" << endl;
        }
        sql = "SELECT id,turn_flag,velocity,scenes,start_x,start_y,end_x,end_y,lane_num from "+tablename+" where id=" + to_string(id);
        nontransaction N(C);
        result R( N.exec( sql ));
        for (result::const_iterator c = R.begin(); c != R.end(); ++c)
        {
            lane.current_lane_id = c[0].as<int>();

            if (ID_SUM==1){     
                lane.next_lane_id = 1;
            }
            lane.LanePoints_right_current_line = get_lanelink_point_zqy(lane.current_lane_id, "lanelink_rightpoints_"+address);
            lane.LanePoints_left_current_line = get_lanelink_point_zqy(lane.current_lane_id, "lanelink_leftpoints_"+address);
            lane.ReferencePoints_current_line = get_referencr_point_zqy(lane.current_lane_id);
            lane.turn_flag = c[1].as<int>();
            lane.current_lane_num = c[8].as<int>();
            lane.v = c[2].as<int>();
            lane.scenes_id = c[3].as<int>();
            lane.current_lane_start_point.x = c[4].as<double>();
            lane.current_lane_start_point.y = c[5].as<double>();
            lane.current_lane_end_point.x = c[6].as<double>();
            lane.current_lane_end_point.y = c[7].as<double>();

        }
        cout << "Operation done successfully" << endl;
        C.disconnect ();
    }
    catch (const std::exception &e)
    {
        cerr << e.what() << std::endl;
    }
    return lane;
}

void pub_refer_line(void *dora_context)
{
    cout << "************REFER LINE************" << endl;
    json j;
    json data_x = json::array();
    json data_y = json::array();
    int num = 0;
    for(int i = 0; i < lane.ReferencePoints_current_line.size(); i++)
    {
        num++;
        data_x.push_back(lane.ReferencePoints_current_line[i].x);
        data_y.push_back(lane.ReferencePoints_current_line[i].y);
    }

    cout << "size : " << num << endl;

    j["seq"] = 1;
    j["size"] = num;
    j["frame_id"] = "refer_lane";
    j["x"] = data_x;
    j["y"] = data_y;

    int count = 0;
    for(const auto& x : data_x){
        count ++;
        cout << "x: " << x << " " << count << endl;
    }

    std::string json_string = j.dump(4); 
    char *c_json_string = new char[json_string.length() + 1];
    strcpy(c_json_string, json_string.c_str());
    std::string out_id = "refer_lane";
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), c_json_string, std::strlen(c_json_string));

    if (result != 0)
    {
        std::cerr << "failed to send output" << std::endl;
    }
}

void pub_right_line(void *dora_context)
{
    cout << "************RIGHT LINE************" << endl;
    json j;
    json data_x = json::array();
    json data_y = json::array();
    int num = 0;
    for(int i = 0; i < lane.LanePoints_right_current_line.size(); i++)
    {
        num++;
        data_x.push_back(lane.LanePoints_right_current_line[i].x);
        data_y.push_back(lane.LanePoints_right_current_line[i].y);
    }

    cout << "size : " << num << endl;

    j["seq"] = 1;
    j["size"] = num;
    j["frame_id"] = "right_lane";
    j["x"] = data_x;
    j["y"] = data_y;

    int count = 0;
    for(const auto& x : data_x){
        count ++;
        cout << "x: " << x << " " << count << endl;
    }

    std::string json_string = j.dump(4); 
    char *c_json_string = new char[json_string.length() + 1];
    strcpy(c_json_string, json_string.c_str());
    std::string out_id = "right_lane";
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), c_json_string, std::strlen(c_json_string));

    if (result != 0)
    {
        std::cerr << "failed to send output" << std::endl;
    }
}
// https://blog.csdn.net/weixin_73062623/article/details/130169207#/
void pub_left_line(void *dora_context)
{
    cout << "************LEFT LINE************" << endl;
    json j;
    json data_x = json::array();
    json data_y = json::array();
    int num = 0;
    for(int i = 0; i < lane.LanePoints_left_current_line.size(); i++)
    {
        num++;
        //cout << "x: " << lane.LanePoints_left_current_line[i].x << " y: " << lane.LanePoints_left_current_line[i].y << " " << num << endl;

        // j["x"] = lane.LanePoints_left_current_line[i].x;
        // j["y"] = lane.LanePoints_left_current_line[i].y;

        data_x.push_back(lane.LanePoints_left_current_line[i].x);
        data_y.push_back(lane.LanePoints_left_current_line[i].y);
    }
    cout << "size : " << num << endl;
    j["seq"] = 1;
    j["size"] = num;
    j["frame_id"] = "left_lane";
    j["x"] = data_x;
    j["y"] = data_y;

    int count = 0;
    for(const auto& x : data_x){
        count ++;
        cout << "x: " << x << " " << count << endl;
    }
    cout<<j<<endl;

    std::string json_string = j.dump(4); 
    char *c_json_string = new char[json_string.length() + 1];
    strcpy(c_json_string, json_string.c_str());
    std::string out_id = "left_lane";
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), c_json_string, std::strlen(c_json_string));

    if (result != 0)
    {
        std::cerr << "failed to send output" << std::endl;
    }
}

int run(void *dora_context)
{
    unsigned char counter = 0;

    for(int i = 0; ; i++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            select_Lane_by_id(1);
            pub_refer_line(dora_context);
            pub_right_line(dora_context);
            pub_left_line(dora_context);
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
    std::cout << "HELLO MAP" << std::endl;

    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "GOODBYE MAP" << std::endl;

    return ret;
}
