// include statements
#include <iostream>
#include <cstddef>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <cmath>
#include <random>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include "matplotlibcpp.h"

// Node definiton 
struct Node{
    // x and y position of the node
    float x;
    float y;
    // Parent Node to the current Node
    Node *parent;
};

//array of all nodes
Node node_list[1000];  

// counter to keep track of no of elements in node_list 
int count=0;

// for finding the final path
Node location[500];

// for backtracking
int backtrack = 0;

//Obstcale Points
//change point to change the location of obstacle
int obstacle_array[][2]={{3, 7}, {7, 7}, {6, 4}, {4, 4}, {4, 6}, {3, 6}};

//  sampling probablity to set rate of sampling
float sampling_probablity;
// variable for sampling radius
float sampling_radius;

// function to sample random points, the rate of sampling is biased by using a sampling probablity 
Node sampler(int goal_x, int goal_y, float prob)
{
    std::random_device rd; //Will be used to obtain a seed for the random number engine
    std::mt19937 generator(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<float> distribution_biased(0.0, 1.0);
    std::uniform_real_distribution<float> distribution_x(0.0, 15.0);
	std::uniform_real_distribution<float> distribution_y(0.0, 15.0);
    float random_prob = distribution_biased(generator);
    Node sample_node={};
    if(random_prob>prob)
    {   
        sample_node.x= distribution_x(generator);
        sample_node.y= distribution_y(generator);
        sample_node.parent= NULL;
    }
    else
    {
        sample_node.x= goal_x;
        sample_node.y= goal_y;
        sample_node.parent= NULL;
    }

    return sample_node;
}

//finds distance between two Node's
float distance(Node node1,Node node2)
{
    float dist=sqrt(((node1.x-node2.x)*(node1.x-node2.x))+((node1.y-node2.y)*(node1.y-node2.y)));
    return dist;
}

//function to find the nearest node
int find_nearest(Node current)
{
    int nearest_node;
    float minimum_dist= 20;
    for(int i=0;i<count;i++)
    {
        float dist=distance(current,node_list[i]);
        if(dist<minimum_dist)
        {
            minimum_dist=dist;
            nearest_node=i;
        }
    }

    return nearest_node;

}

//checks intersection between obstacles and path
bool check_collision(Node end1, Node end2)
{   

    typedef boost::geometry::model::d2::point_xy<double> point_xy;
    typedef boost::geometry::model::polygon<point_xy> polygon_t;
    typedef boost::geometry::model::linestring<point_xy> linestring_t;

    polygon_t obstacle;
    linestring_t ls1;

    std::vector< point_xy > pointls; 

    point_xy point1(end1.x , end1.y);
    pointls.push_back( point1 );

    point_xy point2(end2.x , end2.y);
    pointls.push_back( point2 );

    boost::geometry::assign_points( ls1, pointls );

    std::vector< point_xy > ObstaclePoints; 

    for(int i=0;i<(sizeof(obstacle_array)/sizeof(obstacle_array[0]));i++)
    {   
        point_xy point(obstacle_array[i][0] ,obstacle_array[i][1]);
        ObstaclePoints.push_back( point );
    }
    boost::geometry::assign_points( obstacle, ObstaclePoints);
    bool collision= boost::geometry::intersects(ls1, obstacle);

    return collision;
}

//plotting function
void plotting(bool sucess)
{   
    namespace plt = matplotlibcpp;
    //plot obstacle
    std::vector<double> x = {};
    std::vector<double> y = {};

    for(int i=0;i<(sizeof(obstacle_array)/sizeof(obstacle_array[0]));i++)
    {
        x.push_back(obstacle_array[i][0]);
        y.push_back(obstacle_array[i][1]);
    }
    x.push_back(obstacle_array[0][0]);
    y.push_back(obstacle_array[0][1]);

    plt::plot(x, y,"k");
    
    //plot of all nodes
    std::vector<double> scatter_x = {};
    std::vector<double> scatter_y = {};
    
    for(int i=0;i<count;i++)
    {  
        
            scatter_x.push_back(node_list[i].x);
            scatter_y.push_back(node_list[i].y);
       
    }

    plt::plot(scatter_x, scatter_y,".r");

    //plot tree
    for(int i=0;i<count;i++)
    {   
        if(node_list[i].parent != NULL)
        {
            std::vector<double> node_x = {};
            std::vector<double> node_y = {};
            node_x.push_back(node_list[i].x);
            node_y.push_back(node_list[i].y);
            node_x.push_back(node_list[i].parent->x);
            node_y.push_back(node_list[i].parent->y);
            plt::plot(node_x, node_y,"r");
        }
    }

    if(sucess==1)
    {
        //plot final path from start to goal
        int i = 0;
        while(i<backtrack)
        {   
            if(location[i].parent != NULL)
            {
                std::vector<double> node_x = {};
                std::vector<double> node_y = {};
                node_x.push_back(location[i].x);
                node_y.push_back(location[i].y);
                node_x.push_back(location[i].parent->x);
                node_y.push_back(location[i].parent->y);
                plt::plot(node_x, node_y,".b-");
            }
            i++;
        }
    }
       
    plt::grid(true);
    plt::show();


}

//main planner
bool planner(float start_x, float start_y, float goal_x, float goal_y, float sampling_radius)
{
    //variable to see if goal has been reached
    bool flag = 1;

    // defining the start as a Node
    Node start={start_x,start_y};
    Node goal={goal_x,goal_y};
    Node sample_node={};  
    int nearest_node;  

    //add start to the node_list
    node_list[count]=start;
    count++;

    for (int j=0; j<500;)
    {
        //samples a random point
        sample_node = sampler(goal_x,goal_y,sampling_probablity);

        // finds the nearest node to the sampled point
        nearest_node = find_nearest(sample_node);
        float dist = distance(sample_node,node_list[nearest_node]);

        // checking if tghe sampled node lies inside the sampling radius
        if (dist > sampling_radius)
        {                                                                                                                                                                                                                                                                                                                                                                                                       
         //computing the angle b/w near and rand wrt x-axis
         float angle = atan2(sample_node.y - node_list[nearest_node].y,sample_node.x - node_list[nearest_node].x);                                                                                                                                                                                                                                                                                                                                                                                   

        // modifies sample_node in the direction of nearest node 0.5 distance from it
        sample_node.x=  node_list[nearest_node].x + sampling_radius*cos(angle);
        sample_node.y=  node_list[nearest_node].y + sampling_radius*sin(angle);
        }

        //obstacle checking
        bool collision = check_collision(sample_node, node_list[nearest_node]);
        if(collision==1)
        {
            continue;
        }
        else
        {
            sample_node.parent= &node_list[nearest_node];
            node_list[count]= sample_node;
            count++;
            j++;

        }

        float goal_dist=distance(sample_node,goal);
        if(goal_dist<=0.1)
        {
            goal.parent= &sample_node;
            node_list[count]=goal;
            count++;
            flag = 0;
            //Goal Reached
            break;
        }
    }
    if(flag==1)
    {
        //Goal not found
        return 0;
    }
        
    //backtracking to find the final path from goal to start
    Node *current= &goal;
    while(current->parent != NULL)
    {   
        location[backtrack]=*current;
        backtrack++;
        current=current->parent;
    }

    location[backtrack]=start;
    backtrack++;

    return 1;
}
//starting of the main function
int main()
{   float start_x, start_y, goal_x, goal_y;
    
    // taking inputs from the user
    std::cout<<"Enter sampling probablity : ";
    std::cin>> sampling_probablity;

    std::cout<<"Enter sampling radius (prefered to be less than 1): ";
    std::cin>> sampling_radius;

    std::cout<<"Enter x postion of start : ";
    std::cin>> start_x;
    
    std::cout<<"Enter y postion of start : ";
    std::cin>> start_y;

    std::cout<<"Enter x postion of goal: ";
    std::cin>> goal_x;

    std::cout<<"Enter y postion of goal : ";
    std::cin>> goal_y;

    
    
    
    //calling planner
    bool sucesss = planner(start_x,start_y,goal_x,goal_y,sampling_radius);
    //calling plotting
    plotting(sucesss);

    if(sucesss)
    {
        std::cout<<"goal reached"<<"\n";
    }
    else
    {
        std::cout<<"goal not reached";
    }

    std::cout<<"Total no of nodes = "<<count<<"\n"<<"No of nodes in final path = "<<backtrack<<"\n";

    return 0;
}