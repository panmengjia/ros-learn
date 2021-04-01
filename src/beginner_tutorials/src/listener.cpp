#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

//这是一个回调函数，当有新消息到达chatter话题时它就会被调用。该消息是用boost shared_ptr智能指针传递的，这意味着你可以根据需要存储它，即不用担心它在下面被删除，又不必复制底层（underlying）数据。
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
/**
 * 
 同样地，我们来总结一下：
    初始化ROS系统
    订阅chatter话题
    开始spin自循环，等待消息的到达
    当消息到达后，调用chatterCallback()函数 
*/
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  //通过主节点订阅chatter话题。每当有新消息到达时，ROS将调用chatterCallback()函数。
  //第二个参数是队列大小，以防我们处理消息的速度不够快。
  //在本例中，如果队列达到1000条，再有新消息到达时旧消息会被丢弃。

  //NodeHandle::subscribe()返回一个ros::Subscriber对象，你必须保持它，除非想取消订阅。
  //当Subscriber对象被析构，它将自动从chatter话题取消订阅。
  //还有另一些版本的NodeHandle::subscribe()函数，可以让你指定为类的成员函数，甚至是可以被Boost.Function对象调用的任何函数。
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  //ros::spin()启动了一个自循环，它会尽可能快地调用消息回调函数。不过不要担心，如果没有什么事情，它就不会占用太多CPU。
  //另外，一旦ros::ok()返回false，ros::spin()就会退出，这意味着ros::shutdown()被调用了，主节点让我们关闭（或是因为按下Ctrl+C，它被手动调用）。 
  ros::spin();

  return 0;
}