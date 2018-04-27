// RRT Planning for Visual Servoing
// Written by Deepak Raina, TCS Robotics Lab Noida

/* INPUT:
 * desired and current features (s,sd),
*/

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <fstream>
#include "gen_new_node.h"
#include "gnuplot_ci.h"
#include <stdlib.h>

using namespace std;

int main()
{
    //Current Features
    double centroid[2]={156.641,-122.903};
    double s[2] = {centroid[0], centroid[1]};

    //Desired Features
    double centroid_d[2]={0,0};
    double sd[2] = {centroid_d[0], centroid_d[1]};

    double ds[2]={0,0}; /*Initial Feature Velocity*/
    double new_node_tree1[5] = {0, 0, 0, 0, 0};

    int counter=1; /*counter is set to 0 if it reaches its goal*/
    int n=1; /*n is the number of points or nodes in the tree*/
    int m=3; /*m is the number of features being tracked here*/
    int radius=1; /*If the node reaches in the radius of the goal it comes out of the loop*/
    int i=1; /*counts number of iterations and hence number of node points*/
    int bias=2;

    // Feature tree : tree1
    vector<vector<double> > tree1(5, vector<double> (100000, 0));
    tree1[0][0]=s[0];
    tree1[1][0]=s[1];
    tree1[2][0]=ds[0];
    tree1[3][0]=ds[1];
    tree1[4][0]=n-1;


    //text files
    ofstream st; //start
    ofstream gl; //goal
    ofstream t1; //tree1
    ofstream t1b; //tree1 backtracking
    ofstream t1bv; //tree1 velocitybacktracking

    st.open("start.txt",std::ios_base::trunc);
    gl.open("goal.txt", std::ios_base::trunc);
    t1.open("tree1.txt", std::ios_base::trunc);
    t1b.open("tree1back.txt", std::ios_base::trunc);
    t1bv.open("tree1velback.txt", std::ios_base::trunc);

    cout << "RRT Planning" << endl;
    cout << "-----------------------------------------" << endl;
    st << s[0] << "\t" << s[1] << "\t" << "\n";
    gl << sd[0] << "\t" << sd[1] <<"\t" << "\n";
    t1 << tree1[0][0] << "\t" << tree1[1][0] << "\t" << tree1[2][0] << "\t" << tree1[3][0] << "\t" << tree1[4][0] << "\n";
    cout << "Start feature : " << s[0] << " " << s[1] << endl;
    cout << "Goal feature : " << sd[0] << " " << sd[1] << endl;
    cout << "-----------------------------------------" << endl;
    cout << "Press enter to start planning: " << endl;
    cout << "-----------------------------------------" << endl;
    getchar();

    int dc_violated=0;
    //RRT Loop starts here
    while (counter!=0){
        double dx=pow(tree1[0][n-1]-sd[0],2);
        double dy=pow(tree1[1][n-1]-sd[1],2);
        double d=dx+dy-pow(radius,2); //using area also
        cout << "---------------------------------------------------------" << endl;
        cout << "Distance error = " << d << endl;
        if (d<0){           /*check if the new node is within the vicinity of the goal*/
            counter=0;
        }
        else{
            gen_new_node(i,bias,n,sd,tree1,new_node_tree1,dc_violated);

            if (dc_violated==0)
            {
                n++;
                cout << " " << endl;
                cout << "--------------------------------------------------------" << endl;
                cout << "!!!!!----Node Accepted---!!!!!" << endl;
                cout << "Node No. " << n-1 << endl;
                cout << "New Node in image plane " << new_node_tree1[0] << " " << new_node_tree1[1] << " " << new_node_tree1[2] <<  " " << new_node_tree1[3] << " " << new_node_tree1[4] << endl;
                //getchar();
                tree1[0][n-1]=new_node_tree1[0];
                tree1[1][n-1]=new_node_tree1[1];
                tree1[2][n-1]=new_node_tree1[2];
                tree1[3][n-1]=new_node_tree1[3];
                tree1[4][n-1]=new_node_tree1[4];

                t1 << new_node_tree1[0] << "\t" << new_node_tree1[1] << "\t" << new_node_tree1[2] << "\t" << new_node_tree1[3] << "\t" << new_node_tree1[4] << endl;
            }
            else
            {
                cout << "xxxxx----Node discarded----xxxxxx" << endl;
            }
        }
        i++;
        //        getchar();
    }
    cout << "-----------------------------------------" << endl;
    cout << "Number of Nodes = " << n-1 << endl;
    cout << "-----------------------------------------" << endl;

    // Backtracking of tree1
    // cout << "Press Enter for Backtracking" << endl;
    //getchar();

    cout << "Tree Backtracking (Displaying Node number)" << endl;

    while ((n-1)>=0){
        int ls=tree1[4][n-1];
        cout << ls << endl;

        //writing image trajectory to text file
        t1b << tree1[0][n-1] << "\t" << tree1[1][n-1] << endl;
        t1bv << tree1[2][n-1] << "\t" << tree1[3][n-1] << endl;

        n=ls;
    }

    st.close();
    gl.close();
    t1.close();
    t1b.close();
    t1bv.close();

    cout << "RRT planning ends" << endl;
    cout << "-------------------------------------------" << endl;
    plots();
    cout << "-------------------------------------------" << endl;
}
