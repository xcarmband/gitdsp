// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#define SERVICE_PORT	25000	/* hard-coded port number */
#define BUFSIZE 2048
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <vector>
#include <queue>

#include <stdio.h>
#include <fstream>
#include <ctime>
#include <sys/time.h>
// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>
#include "quaternion/quaternion.c++"
//for UDP
#include <stdlib.h>
//#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>

//using namespace std;


// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
    DataCollector(float sampling_period)
    : onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose(), _accel(), _gyro(),_a_pred()
    {
    }
    
    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        // We've lost a Myo.
        // Let's clean up some leftover state.
        roll_w = 0;
        pitch_w = 0;
        yaw_w = 0;
        onArm = false;
        isUnlocked = false;
    }
    
    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;
        
        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                          1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
        
        // Convert the floating point angles in radians to a scale from 0 to 18.
        roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
        pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
        yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
        _quat = Quaternion<float>(quat.w(), quat.x(), quat.y(), quat.z());
        
        
    }
    
    
    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        currentPose = pose;
        
        if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
            // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
            // Myo becoming locked.
            myo->unlock(myo::Myo::unlockHold);
            
            // Notify the Myo that the pose has resulted in an action, in this case changing
            // the text on the screen. The Myo will vibrate.
            myo->notifyUserAction();
        } else {
            // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
            // are being performed, but lock after inactivity.
            myo->unlock(myo::Myo::unlockTimed);
        }
    }
    
    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
                   myo::WarmupState warmupState)
    {
        onArm = true;
        whichArm = arm;
    }
    
    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
    }
    
    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = true;
    }
    
    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = false;
    }
    
    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.
    
    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
        // Clear the current line
        std::cout << '\r';
        
        // Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
        std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
        << '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
        << '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';
        
        if (onArm) {
            // Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.
            
            // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
            // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
            // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
            std::string poseString = currentPose.toString();
            
            std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
            << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
            << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
        } else {
            // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
            std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
        }
        
        std::cout << std::flush;
    }
    
    void gravity_compensate(){
        float v[3];
        v[0] = _accel.x();
        v[1] = _accel.y();
        v[2] = _accel.z();
        _quat.QuatRotation(v);
        accel_pure = myo::Vector3<float>(v[0],v[1],v[2]-1);
    }
    
    void kalmanfilter_position(float alpha, float gamma){
        //update raw velocity
        posit_raw = myo::Vector3<float>(posit_filter.x() + veloc_filter.x(), posit_filter.y() + veloc_filter.y(), posit_filter.z()+ veloc_filter.z());
        
        
        //Predict next status
        _pos_pred = myo::Vector3<float>(0,0,0);
        //_pos_pred = posit_filter;
        // _v_pred =veloc_filter;
        
        //Predict the covariance matrix
        _posp_pred = myo::Vector3<float>(_posp.x() + alpha, _posp.y() + alpha, _posp.z() + alpha);
        
        //refresh the k
        k = myo::Vector3<float>(_posp_pred.x()/(_posp_pred.x()+gamma), _posp_pred.y()/(_posp_pred.y()+gamma),_posp_pred.z()/(_posp_pred.z()+gamma));
        
        //adjust
        myo::Vector3<float> tmp(posit_raw.x()-_pos_pred.x(),posit_raw.y() - _pos_pred.y(), posit_raw.z() - _pos_pred.z());
        posit_filter = myo::Vector3<float>(_pos_pred.x() + k.x()*tmp.x(),_pos_pred.y()+k.y()*tmp.y(),_pos_pred.z() + k.z()*tmp.z());
        
        //adjust the covariance matrix
        _posp = myo::Vector3<float>((1-gamma)*_posp_pred.x(), (1-gamma)*_posp_pred.y(),(1-gamma)*_posp_pred.z());
        
        
    }
    
    void kalmanfilter_velocity(float alpha, float gamma){
        //update raw velocity
                veloc_raw = myo::Vector3<float>(veloc_filter.x() + accel_filter.x(), veloc_filter.y() + accel_filter.y(), veloc_filter.z()+ accel_filter.z());
        //        veloc_raw = myo::Vector3<float>(veloc_raw.x() + accel_pure.x(), veloc_raw.y() + accel_pure.y(), veloc_raw.z()+ accel_pure.z());
        // veloc_raw = myo::Vector3<float>(veloc_filter.x() + accel_pure.x(), veloc_filter.y() + accel_pure.y(), veloc_filter.z()+ accel_pure.z());
        
        
        //Predict next status
        _v_pred = myo::Vector3<float>(0,0,0);
        //_v_pred =veloc_filter;
        
        //Predict the covariance matrix
        _vp_pred = myo::Vector3<float>(_vp.x() + alpha, _vp.y() + alpha, _vp.z() + alpha);
        
        //refresh the k
        k = myo::Vector3<float>(_vp_pred.x()/(_vp_pred.x()+gamma), _vp_pred.y()/(_vp_pred.y()+gamma),_vp_pred.z()/(_vp_pred.z()+gamma));
        
        //adjust
        myo::Vector3<float> tmp(veloc_raw.x()-_v_pred.x(),veloc_raw.y() - _v_pred.y(), veloc_raw.z() - _v_pred.z());
        veloc_filter = myo::Vector3<float>(_v_pred.x() + k.x()*tmp.x(),_v_pred.y()+k.y()*tmp.y(),_v_pred.z() + k.z()*tmp.z());
        
        //adjust the covariance matrix
        _vp = myo::Vector3<float>((1-gamma)*_vp_pred.x(), (1-gamma)*_vp_pred.y(),(1-gamma)*_vp_pred.z());
        
        
    }
    
    
    void kalmanfilter(float alpha, float gamma){
        //compensate gravity;
        gravity_compensate();
        
        //Predict next status
        _a_pred = accel_filter;
        
        //Predict the covariance matrix
        _p_pred = myo::Vector3<float>(_p.x() + alpha, _p.y() + alpha, _p.z() + alpha);
        
        //refresh the k
        k = myo::Vector3<float>(_p_pred.x()/(_p_pred.x()+gamma), _p_pred.y()/(_p_pred.y()+gamma),_p_pred.z()/(_p_pred.z()+gamma));
        
        //adjust
        myo::Vector3<float> tmp(accel_pure.x()-_a_pred.x(),accel_pure.y() - _a_pred.y(), accel_pure.z() - _a_pred.z());
        accel_filter = myo::Vector3<float>(_a_pred.x() + k.x()*tmp.x(),_a_pred.y()+k.y()*tmp.y(),_a_pred.z() + k.z()*tmp.z());
        
        //adjust the covariance matrix
        _p = myo::Vector3<float>((1-gamma)*_p_pred.x(), (1-gamma)*_p_pred.y(),(1-gamma)*_p_pred.z());
        
        
    }
    
    void onAccelerometerData (myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &accel){
        _accel = accel;
    }
    
    void 	onGyroscopeData (myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &gyro){
        _gyro = gyro;
    }
    
    
    // These values are set by onArmSync() and onArmUnsync() above.
    bool onArm;
    myo::Arm whichArm;
    
    // This is set by onUnlocked() and onLocked() above.
    bool isUnlocked;
    
    // These values are set by onOrientationData() and onPose() above.
    int roll_w, pitch_w, yaw_w;
    myo::Vector3<float> _accel;
    myo::Vector3<float> _gyro;
    // type myo::quaternion turned into quaterion locally defined
    Quaternion<float>  _quat;
    myo::Pose currentPose;
    myo::Vector3<float>     accel_pure;
    myo::Vector3<float>     accel_filter;
    myo::Vector3<float>     veloc_raw;
    myo::Vector3<float>     veloc_filter;
    myo::Vector3<float>     posit_raw;
    myo::Vector3<float>     posit_filter;
    
    myo::Vector3<float>     accel_pred;
    myo::Vector3<float>     veloc_pred;
    myo::Vector3<float>     posit_pred;
    
    void    matrix_print(std::vector<std::vector<float>> in){
        for(int i = 0 ; i < in.size(); i++){
            for(int j = 0 ; j < in[i].size(); j++){
                std::cout<<in[i][j];
            }
            std::cout << std::endl;
        }
    }
    
    std::vector<myo::Vector3<float>>    matrix_mul_vectors(std::vector<std::vector<float>> m, std::vector<myo::Vector3<float>> a){
        std::vector<myo::Vector3<float>> ret(m.size());
        for(int i = 0 ; i < m.size() ; i++){
            ret[0] = myo::Vector3<float>(0,0,0);
            for(int j = 0 ; j<m[i].size() ; j++){
                ret[0] = myo::Vector3<float>( ret[0].x() + m[i][j] * a[j].x(), ret[0].y() + m[i][j] * a[j].y(), ret[0].z() + m[i][j] * a[j].z()) ;
            }
        }
        return ret;
    }
    
private:
    myo::Vector3<float> _a_pred;
    myo::Vector3<float> _p_pred;
    myo::Vector3<float> _p;
    myo::Vector3<float> k;
    myo::Vector3<float> _v_pred;
    myo::Vector3<float> _vp_pred;
    myo::Vector3<float> _vp;
    myo::Vector3<float> _pos_pred;
    myo::Vector3<float> _posp_pred;
    myo::Vector3<float> _posp;
};


int main(int argc, char** argv)
{
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {
        
        // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
        // publishing your application. The Hub provides access to one or more Myos.
        myo::Hub hub("com.example.hello-myo");
        
        std::cout << "Attempting to find a Myo..." << std::endl;
        
        // Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
        // immediately.
        // waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
        // if that fails, the function will return a null pointer.
        myo::Myo* myo = hub.waitForMyo(10000);
        
        // If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
        if (!myo) {
            throw std::runtime_error("Unable to find a Myo!");
        }
        
        // We've found a Myo.
        std::cout << "Connected to a Myo armband!" << std::endl << std::endl;
        float sampling_period = 0.02;
        
        // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
        DataCollector collector(sampling_period);
        
        // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
        // Hub::run() to send events to all registered device listeners.
        hub.addListener(&collector);
        ofstream    myfile;
        
        timeval t1, t2;
        gettimeofday(&t1,NULL);
        
        // Finally we enter our main loop.
        system("rm /Users/tibenche/Google Drive/workspace/TCXJ/gitdsp/data.txt");
        //
        struct sockaddr_in myaddr;	/* our address */
        struct sockaddr_in remaddr;	/* remote address */
        socklen_t addrlen = sizeof(remaddr);		/* length of addresses */
        int recvlen;			/* # bytes received */
        int fd;				/* our socket */
        unsigned char buf[BUFSIZE];	/* receive buffer */
        
        
        /* create a UDP socket */
        
        if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            perror("cannot create socket\n");
            return 0;
        }
        
        /* bind the socket to any valid IP address and a specific port */
        
        memset((char *)&myaddr, 0, sizeof(myaddr));
        myaddr.sin_family = AF_INET;
        myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
        myaddr.sin_port = htons(SERVICE_PORT);
//        bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr));
        
        if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
            perror("bind failed");
            return 0;
        }
        
        /* now loop, receiving data and printing what we received */
        for (;;) {
            printf("waiting on port %d\n", SERVICE_PORT);
            recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
            printf("received %d bytes\n", recvlen);
            if (recvlen > 0) {
                buf[recvlen] = 0;
                printf("received message: \"%s\"\n", buf);
            }
        }
        while (1) {
            // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
            // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
            //hub.run(1000/20);
            hub.run(int(sampling_period*1000));
            // After processing events, we call the print() member function we defined above to print out the values we've
            // obtained from any events that have occurred.
            
            collector.print();
            collector.kalmanfilter(0.001, 0.1);
            collector.kalmanfilter_velocity(0.1, 0.2);
            collector.kalmanfilter_position(0.1, 0.2);
            
            
            myfile.open("/Users/tibenche/Google Drive/workspace/TCXJ/gitdsp/data.txt",ios::app);
            //print to file
            gettimeofday(&t2,NULL);
            double timeuse = t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec)/1000000.0;
            myfile << timeuse << '\t';
            //myfile << difftime(current_time, start_time) << '\t';
            myfile << collector._accel.x() << '\t';
            myfile << collector._accel.y() << '\t';
            myfile << collector._accel.z() << '\t';
            myfile << collector.accel_pure.x() << '\t';
            myfile << collector.accel_pure.y() << '\t';
            myfile << collector.accel_pure.z() << '\t';
            myfile << collector.accel_filter.x() << '\t';
            myfile << collector.accel_filter.y() << '\t';
            myfile << collector.accel_filter.z() << '\t';
            myfile << collector.veloc_raw.x() << '\t';
            myfile << collector.veloc_raw.y() << '\t';
            myfile << collector.veloc_raw.z() << '\t';
            myfile << collector.veloc_filter.x() << '\t';
            myfile << collector.veloc_filter.y() << '\t';
            myfile << collector.veloc_filter.z() << '\t';
            myfile << collector.posit_raw.x() << '\t';
            myfile << collector.posit_raw.y() << '\t';
            myfile << collector.posit_raw.z() << '\t';
            myfile << collector.posit_filter.x() << '\t';
            myfile << collector.posit_filter.y() << '\t';
            myfile << collector.posit_filter.z() << '\n';
            
            
            myfile.close();
        }
        
        
        // If a standard exception occurred, we print out its message and exit.
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
        
    }
    
}
