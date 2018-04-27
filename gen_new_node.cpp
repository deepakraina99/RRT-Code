#include<iostream>
#include<cstdlib>
#include<stdio.h>
#include<cmath>
#include<vector>
#include<algorithm>
#include<ctime>
#include"gen_new_node.h"

using namespace std;

void gen_new_node(int i, int bias, int n, double sd[], std::vector< std::vector<double> > tree1, double new_node_tree1[], int &dc_violated){

    //Set parameters
    double delta = 1;
    double diff1, diff2, nx, ny, na, norm, rn1, rn2, rn3;

    if (i%10>=bias){
        srand(time(NULL)*i);
        int max, min;

        max=320;
        min=-320;
        rn1 = (rand()%(max*1000-min*1000+1)+min*1000)/1000.0; //Random number between -320 & 320
        //y
        max=240;
        min=-240;
        rn2 = (rand()%(max*1000-min*1000+1)+min*1000)/1000.0; //Random number between -240 & 240

        cout << "Random Feature " << rn1 <<  " " << rn2 << endl;
        //getchar();
    }
    else{
        rn1=sd[0];
        rn2=sd[1];
        cout << "Biased Feature " << rn1 <<  " " << rn2 << endl;
    }

    double r[3]={rn1,rn2};

    //finding nearest node to the random node generated
    std::vector<double> d_frm_ran(n);

    for (int i=0; i<=n-1; i=i+1){
        double t1=tree1[0][i];
        double t2=tree1[1][i];
        double t[2]={t1, t2};

        nx=pow(t[0]-r[0],2);
        ny=pow(t[1]-r[1],2);
        norm = sqrt(nx+ny); //excluding area

        //Direction criteria

        double dpx, dpy, dpa, dp_norm, drx, dry, dra, dr_norm, dp[3],dr[3], dpr[3];

        drx=r[0]-t[0];
        dry=r[1]-t[1];
        dr_norm=sqrt(pow(drx,2)+pow(dry,2));
        dr[0]=drx/dr_norm;
        dr[1]=dry/dr_norm;
        //        cout << "drxya " << drx << " " << dry << " " << dra << endl;
        //        cout << "dr " << dr[0] << " " << dr[1] << " " << dr[2] << endl;

        if (i==0){
            dpx=tree1[0][i]-0;
            dpy=tree1[1][i]-0;
            dp_norm=sqrt(pow(dpx,2)+pow(dpy,2));
            dp[0]=dpx/dp_norm;
            dp[1]=dpy/dp_norm;
            //            cout << "dpxya " << dpx << " " << dpy << " " << dpa << endl;
            //            cout << "dp " << dp[0] << " " << dp[1] << " " << dp[2] << endl;

        }
        else {
            dpx=tree1[0][i]-tree1[0][tree1[4][i-1]];
            dpy=tree1[1][i]-tree1[1][tree1[4][i-1]];
            dp_norm=sqrt(pow(dpx,2)+pow(dpy,2));
            dp[0]=dpx/dp_norm;
            dp[1]=dpy/dp_norm;
            //            cout << "dpxya " << dpx << " " << dpy << " " << dpa << endl;
            //            cout << "dp " << dp[0] << " " << dp[1] << " " << dp[2] << endl;
        }
        dpr[0]=abs(dr[0]-dp[0]);
        dpr[1]=abs(dr[1]-dp[1]);
        //        cout << "dpr " << dpr[0] << " " << dpr[1] << " " << dpr[2] << endl;

        double dc_limit = sin(M_PI/10);
        //        cout << "DC limit = sin(M_PI/10) = " <<  dc_limit << endl;

        if ((dpr[0] < dc_limit) && (dpr[1] < dc_limit)) //sin(M_PI/10) = 0.309
        {
            //            cout << "DC satisfied" << endl;
            d_frm_ran[i]=norm;
        }
        else{
            //            cout << "DC not satisfied" << endl;
            d_frm_ran[i]=1000000000;
        }

//        d_frm_ran[i]=norm;
    }
    //    for (int i = 0; i < d_frm_ran.size(); i++) {
    //            cout << "d_frm_ran " << d_frm_ran[i] << "\n";
    //        }
    double min_pos = distance(d_frm_ran.begin(),min_element(d_frm_ran.begin(),d_frm_ran.end()));
    double temp=*min_element(d_frm_ran.begin(),d_frm_ran.end());
    int ix=min_pos;

    if (temp == 1000000000){
        dc_violated = 1; //all nodes violated direction criteria
        cout << "DC not satisfied" << endl;

    }
    else{
        dc_violated = 0;
    }

    // Parameters
    double lambda=0.1; /*convergence rate*/

    //position and velocity of parent feature vector
    double sp[2]={tree1[0][ix], tree1[1][ix]};
    double dsp[2]={tree1[2][ix], tree1[3][ix]};

    //error between random and parent feature vector
    double e1, e2;
    e1=sp[0]-r[0];
    e2=sp[1]-r[1];

    //feature velocity
    double dsnew1, dsnew2;
    dsnew1 = -lambda*e1 ;
    dsnew2 = -lambda*e2 ;

    //New feature position from RRT
    double snew1, snew2;
    snew1 =  sp[0]+dsnew1*delta;
    snew2 =  sp[1]+dsnew2*delta;

    //Output
    cout << "Parent Node index " << ix << endl;

    cout << "Random Node RRT "<< r[0] << " " << r[1] << endl;

    cout << "Parent Feature Vec " << sp[0] << " " << sp[1] << endl;

    cout << "Error " << e1 << " " << e2 << endl;

    cout << "Parent Feature Velocity " << dsnew1 << " " << dsnew2 << endl;

    cout << "New Feature Vec " << snew1 << " " << snew2 << endl;

    //        getchar();

    new_node_tree1[0]=snew1;
    new_node_tree1[1]=snew2;
    new_node_tree1[2]=dsnew1;
    new_node_tree1[3]=dsnew2;
    new_node_tree1[4]=ix+1;
}




