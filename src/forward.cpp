#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <motionlibrary/ForwardAction.h>
#include <dynamic_reconfigure/server.h>
#include <motionlibrary/forwardServerConfig.h>

typedef actionlib::SimpleActionServer<motionlibrary::ForwardAction> Server;

std_msgs::Int32 pwm;
std_msgs::Int32 dir;
float x;
class forwardAction{
	private:
		ros::NodeHandle nh_;
		Server forwardServer_;
		std::string action_name_;
		motionlibrary::ForwardFeedback feedback_;
		motionlibrary::ForwardResult result_;
		ros::Subscriber sub_;
		float timeSpent, motionTime;
		bool success;
//ROS was not working properly if these variables were declared inside function. Really wierd problem need to do somthing about it 
		ros::Publisher PWM, direction;
		float p, i, d;

	public:
		forwardAction(std::string name):
			//here we are defining the server, third argument is optional
	    	forwardServer_(nh_, name, boost::bind(&forwardAction::analysisCB, this, _1), false),
    		action_name_(name)
		{
			p=0;
			i=0;
			d=0;
//			forwardServer_.registerGoalCallback(boost::bind(&forwardAction::goalCB, this));
			forwardServer_.registerPreemptCallback(boost::bind(&forwardAction::preemptCB, this));
			PWM = nh_.advertise<std_msgs::Int32>("PWM",1000);
			direction = nh_.advertise<std_msgs::Int32>("direction",1000);
            //sub_=nh_.subscribe<std_msgs::Float64>("x",1000,&forwardAction::xcB);
//this type callback can be used if we want to do the callback from some specific node
//			sub_ = nh_.subscribe("name of the node", 1, &forwardAction::analysisCB, this);
			forwardServer_.start();
		}


		~forwardAction(void){
		}

//Some strange warning was occuring if we were using this goalCB function. Aprntly there was some other callback function for goals
			// void goalCB(){
			// 	timeSpent = 0;
			// 	motionTime = forwardServer_.acceptNewGoal()->MotionTime;
			// 	ROS_INFO("%s: New goal recieved %f", action_name_.c_str(),motionTime);
			// }

		void preemptCB(void){

			pwm.data = 0;
			dir.data = 5;
			PWM.publish(pwm);
			direction.publish(dir);							
			ROS_INFO("pwm send to arduino %d in %d", pwm.data,dir.data);

			//this command cancels the previous goal
			// forwardServer_.setPreempted();
		}

		void analysisCB(const motionlibrary::ForwardGoalConstPtr goal){
			ROS_INFO("Inside analysisCB");

			//pwm.data = 90;
			dir.data = 1;
			ros::Rate looprate(1);
			success = true;

			if (!forwardServer_.isActive())
				return;
			float i_temp=0,error_old=0;
			float dt=0.01;

			for(timeSpent=0; timeSpent <= goal->MotionTime; timeSpent+=dt){
				if (forwardServer_.isPreemptRequested() || !ros::ok())
				{
					ROS_INFO("%s: Preempted", action_name_.c_str());
			        // set the action state to preempted
        			forwardServer_.setPreempted();
        			success = false;
        			break;
				}
				// publish the feedback
				feedback_.TimeRemaining = goal->MotionTime - timeSpent;
				float setpoint=goal->MotionTime - timeSpent;//abhi dimension sahi karni hai
				const int pwm_max=255, pwm_min =0,i_max=255,i_min=0;
				//float dt=0.01;
				float p_term,i_term,d_term;
				float error=setpoint-x;
				if(error<0.1 && error >-0.1)
					break;
				else
				{
					int scale=1000;
			        error=error*scale;
			        p_term=error;
			        i_term=i_temp+error*dt;
			        d_term=(error-error_old)/dt;
			        if(i_term>i_max)
				    i_term=i_max;
			        else if(i_term<i_min)
				    i_term=i_min;
			        i_temp=i_term;
			        error_old=error;
			        pwm.data=int((p*p_term+i*i_term+d*d_term)/scale);
			        if(pwm.data<0){
				    dir.data=2;pwm.data=-pwm.data;}
			        if(pwm.data>pwm_max)
				    pwm.data=pwm_max;
			        else if(pwm.data<pwm_min)
				    pwm.data=pwm_min;
				    }

				

/*

{
	
		int pwm;
		float p_term,i_term,d_term;
		float z=Readsensordata;//function to read sensor data.
		float error=setpoint-z;
		if(error<0.1&&error>-0.1)
			write(0);
		else{	
			
			write(pwm);//function to send pwm value.
			delay(dt);//delay function to cause delay			
		}
	}
}*/
				forwardServer_.publishFeedback(feedback_);
				PWM.publish(pwm);
				direction.publish(dir);
				ROS_INFO("pwm send to arduino %d in %d", pwm.data,dir.data);

				ROS_INFO("timeSpent %f", timeSpent);
				ros::spinOnce();
				looprate.sleep();				
			}
			if(success){
				result_.MotionCompleted = success;
				ROS_INFO("pwm send to arduino %d in %d", pwm.data,dir.data);
				ROS_INFO("%s: Succeeded", action_name_.c_str());
				pwm.data = 120;
				dir.data = 5;
				PWM.publish(pwm);
				direction.publish(dir);				
				// set the action state to succeeded
				forwardServer_.setSucceeded(result_);
			}
		}

		void setPID(float new_p, float new_i, float new_d) {
			p=new_p;
			i=new_i;
			d=new_d;
		}
};

forwardAction* forwardObject; 

//dynamic reconfig 
void callback(motionlibrary::forwardServerConfig &config, double level) {
	ROS_INFO("Reconfigure Request: p= %f i= %f d=%f", config.p, config.i, config.d);
	forwardAction &temp = *forwardObject;
	forwardObject->setPID(config.p, config.i, config.d);
}

		void xcB(std_msgs:: Float64 msg)
		{
			x=msg.data; //assuming x is given by the imu

		}
int main(int argc, char** argv){
	ros::init(argc, argv, "forward");
	ros:: NodeHandle n1;
	ros:: Subscriber sub1=n1.subscribe<std_msgs::Float64 >("x",1000,&xcB);
	ROS_INFO("Waiting for Goal");
	

	//register dynamic reconfig server.
	dynamic_reconfigure::Server<motionlibrary::forwardServerConfig> server;
	dynamic_reconfigure::Server<motionlibrary::forwardServerConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	forwardObject = new forwardAction(ros::this_node::getName());

	ros::spin();
	return 0;
}