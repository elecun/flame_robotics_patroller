
#include "basler.gige.cam.grabber.hpp"
#include <flame/log.hpp>
#include <flame/config_def.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace flame;
using namespace std;
using namespace cv;

/* create component instance */
static basler_gige_cam_grabber* _instance = nullptr;
flame::component::object* create(){ if(!_instance) _instance = new basler_gige_cam_grabber(); return _instance; }
void release(){ if(_instance){ delete _instance; _instance = nullptr; }}


bool basler_gige_cam_grabber::on_init(){

    try{

        /* get parameters from profile */
        json parameters = get_profile()->parameters();

        /* read profile */
        _use_image_stream_monitoring.store(parameters.value("use_image_stream_monitoring", false));
        _use_image_stream.store(parameters.value("use_image_stream", false));

        /* pylon initialize */
        PylonInitialize();

        /* find GigE cameras in same netwrok */
        CTlFactory& tlFactory = CTlFactory::GetInstance();
        DeviceInfoList_t devices;
        tlFactory.EnumerateDevices(devices);
        if(devices.size()>=1)
            logger::info("[{}] Found {} cameras", get_name(), devices.size());

        /* create device & insert to device container */
        for(int idx=0;idx<(int)devices.size();idx++){
            _device_map.insert(make_pair(stoi(devices[idx].GetUserDefinedName().c_str()), new CBaslerUniversalInstantCamera(tlFactory.CreateDevice(devices[idx]))));
            logger::info("[{}] Found Camera ID {}, (SN:{}, Address : {})", get_name(), devices[idx].GetUserDefinedName().c_str(), devices[idx].GetSerialNumber().c_str(), devices[idx].GetIpAddress().c_str());
        }

        /* device control handle assign for each camera */
        for(const auto& camera:_device_map){
            _camera_grab_worker[camera.first] = thread(&basler_gige_cam_grabber::_image_stream_task, this, camera.first, camera.second);
            logger::info("[{}] Camera #{} Grabber is running...", get_name(), camera.first);
        }

    }
    catch(const GenericException& e){
        logger::error("[{}] Pylon Generic Exception : {}", get_name(), e.GetDescription());
        return false;
    }
    catch(json::exception& e){
        logger::error("Profile Error : {}", e.what());
        return false;
    }

    return true;
}

void basler_gige_cam_grabber::on_loop(){

}


void basler_gige_cam_grabber::on_close(){

    /* stop grabbing (must be first!!!) */
    for_each(_device_map.begin(), _device_map.end(), [](auto& camera){
        camera.second->StopGrabbing();
    });

    /* work stop signal */
    _use_image_stream_monitoring.store(false);
    _use_image_stream.store(false);
    _worker_stop.store(true);

    /* stop camera grab workers */
    for_each(_camera_grab_worker.begin(), _camera_grab_worker.end(), [](auto& t) {
        if(t.second.joinable()){
            t.second.join();
            logger::info("- Camera #{} Grabber is now stopped", t.first);
        }
    });

    _camera_grab_worker.clear();

    /* camera close and delete */
    for(auto& camera:_device_map){
        if(camera.second->IsOpen()){
            camera.second->Close();
            delete camera.second;
        }
    }

    PylonTerminate();
}

void basler_gige_cam_grabber::on_message(){
    
}


void basler_gige_cam_grabber::_image_stream_task(int camera_id, CBaslerUniversalInstantCamera* camera){
    try{
        camera->Open();

        json parameters = get_profile()->parameters();
        json dataport_config = get_profile()->dataport();

        /* read config */
        string acquisition_mode = parameters.value("acquisition_mode", "Continuous"); // Continuous, SingleFrame, MultiFrame
        double acquisition_fps = parameters.value("acquisition_fps", 30.0);
        string trigger_selector = parameters.value("trigger_selector", "FrameStart");
        string trigger_mode = parameters.value("trigger_mode", "Off");
        string trigger_source = parameters.value("trigger_source", "Line2");
        string trigger_activation = parameters.value("trigger_activation", "RisingEdge");
        int heartbeat_timeout = parameters.value("heartbeat_timeout", 5000);
        string id_str = fmt::format("{}",camera_id);
        string image_stream_port = fmt::format("image_stream_{}", camera_id);
        string image_stream_monitor_port = fmt::format("image_stream_monitor_{}", camera_id); //portname = topic

        int monitoring_width = 0;
        int monitoring_height = 0;
        string monitoring_topic {""};
        try{
            if(parameters.contains(image_stream_monitor_port)){

                monitoring_width = dataport_config.at(image_stream_monitor_port).at("resolution").value("width", 640);
                monitoring_height = dataport_config.at(image_stream_monitor_port).at("resolution").value("height", 480);
                monitoring_topic = fmt::format("{}/{}", get_name(), image_stream_monitor_port);
                logger::info("[{}] Camera #{} monitoring image resolution : {}x{}", get_name(), camera_id, monitoring_width, monitoring_height);
            }
        }
        catch(const json::exception& e){
            logger::error("[{}] Camera #{} monitoring image resolution error : {}", get_name(), camera_id, e.what());
        }

        
        // camera exposure time set (initial)
        for(auto& param:parameters["cameras"]){
            int id = param["id"].get<int>();
            if(id==camera_id){
                double exposure_time = param.value("exposure_time", 100.0);
                CEnumerationPtr(camera->GetNodeMap().GetNode("ExposureAuto"))->FromString("Off");
                CFloatParameter exposureTime(camera->GetNodeMap(), "ExposureTime");
                if(exposureTime.IsWritable()) {
                    exposureTime.SetValue(exposure_time);
                    logger::info("[{}] Camera #{} Exposure Time set : {}", get_name(), camera_id, exposure_time);
                }
            }
        }

        /* camera setting parameters notification */
        logger::info("[{}]* Camera Acquisition Mode : {} (Continuous|SingleFrame)", get_name(), acquisition_mode);
        logger::info("[{}]* Camera Acqusition Framerate : {}", get_name(), acquisition_fps);
        logger::info("[{}]* Camera Trigger Mode : {}", get_name(), trigger_mode);
        logger::info("[{}]* Camera Trigger Selector : {}", get_name(), trigger_selector);
        logger::info("[{}]* Camera Trigger Activation : {}", get_name(), trigger_activation);
        
        /* set camera parameters */
        camera->AcquisitionMode.SetValue(acquisition_mode.c_str());
        camera->AcquisitionFrameRate.SetValue(acquisition_fps);
        camera->AcquisitionFrameRateEnable.SetValue(false);
        camera->TriggerSelector.SetValue(trigger_selector.c_str());
        camera->TriggerMode.SetValue(trigger_mode.c_str());
        camera->TriggerSource.SetValue(trigger_source.c_str());
        camera->TriggerActivation.SetValue(trigger_activation.c_str());
        camera->GevHeartbeatTimeout.SetValue(heartbeat_timeout);

        /* start grabbing */
        camera->StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByUser);
        CGrabResultPtr ptrGrabResult;

        logger::info("[{}] Camera #{} grabber is now running...",get_name(), camera_id);
        unsigned long long camera_grab_counter = 0;
        while(!_worker_stop.load()){
            try{
                if(!camera->IsGrabbing())
                    break;
                
                bool success = camera->RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException); //trigger mode makes it blocked
                if(!success){
                    logger::warn("[{}] Camera #{} will be terminated by force.", get_name(), camera_id);
                    break;
                }
                else { // no timeout, success
                    if(ptrGrabResult.IsValid()){
                        if(ptrGrabResult->GrabSucceeded()){
    
                            auto start = std::chrono::system_clock::now();
    
                            /* grabbed imgae stores into buffer */
                            const uint8_t* pImageBuffer = (uint8_t*)ptrGrabResult->GetBuffer();
    
                            /* get image properties */
                            size_t size = ptrGrabResult->GetWidth() * ptrGrabResult->GetHeight();
                            cv::Mat image(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, (void*)pImageBuffer);
    
                            /* push image into image_stream pipeline  */
                            if(_use_image_stream.load()){

                                //jpg encoding
                                std::vector<unsigned char> encoded_image;
                                cv::imencode(".jpg", image, encoded_image);
    
                                pipe_data msg_image(encoded_image.data(), encoded_image.size());
    
                                if(get_port(image_stream_port)->handle()!=nullptr){
                                    zmq::multipart_t msg_multipart_image_stream;
                                    msg_multipart_image_stream.addstr(id_str);
                                    msg_multipart_image_stream.addmem(encoded_image.data(), encoded_image.size());
                                    msg_multipart_image_stream.send(*get_port(image_stream_port), ZMQ_DONTWAIT);
                                }
                                else{
                                    logger::warn("[{}] {} socket handle is not valid ", get_name(), camera_id);
                                }
                            }
    
    
                            /* publish for monitoring (size reduction for performance)*/
                            if(_use_image_stream_monitoring.load()){
                                
                                cv::Mat monitor_image;
                                cv::resize(image, monitor_image, cv::Size(monitoring_width, monitoring_height));

                                std::vector<unsigned char> encoded_monitor_image;
                                cv::imencode(".jpg", monitor_image, encoded_monitor_image);
                                pipe_data msg_monitor_image(encoded_monitor_image.data(), encoded_monitor_image.size());
    
                                zmq::multipart_t msg_multipart_stream_monitor;
                                msg_multipart_stream_monitor.addstr(monitoring_topic);
                                msg_multipart_stream_monitor.addstr(id_str);
                                msg_multipart_stream_monitor.addmem(encoded_monitor_image.data(), encoded_monitor_image.size());
                                msg_multipart_stream_monitor.send(*get_port(image_stream_monitor_port), ZMQ_DONTWAIT);
                            }
                        }
                        else{
                            logger::warn("[{}] Error-code({}) : {}", get_name(), ptrGrabResult->GetErrorCode(), ptrGrabResult->GetErrorDescription().c_str());
                        }
                    }
                }
                
            }
            catch(Pylon::RuntimeException& e){
                logger::error("[{}] Camera {} Runtime Exception ({})", get_name(), camera_id, e.what());
                break;
            }
            catch(const Pylon::GenericException& e){
                logger::error("[{}] Camera {} Generic Exception ({})", get_name(), camera_id, e.what());
                break;
            }
            catch(const zmq::error_t& e){
                logger::error("[{}] {}", get_name(), e.what());
                break;
            }
        }

        /* stop grabbing */
        camera->StopGrabbing();
        camera->Close();
        logger::info("[{}] Camera #{} grabber is now closed", get_name(), camera_id);
    }
    catch(const GenericException& e){
        logger::error("[{}] {}", get_name(), e.GetDescription());
    }
}


