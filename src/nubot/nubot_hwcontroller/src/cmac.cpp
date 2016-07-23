/* CMAC - Cerebellum Model Articulation Controller */

#include <stdio.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <memory.h>
#include "nubot/nubot_hwcontroller/cmac.h"
#include <assert.h>


/*
MODCMAC returns the output of the network for the input IP after modifying
the weights using the LMS equivalent training algorithm.

The form of the function MODCMAC is
            OP = MODCMAC(WTS,IP,TARGET,BETA,IPRANGE,C,WIDTH,MEMSIZE)
where
TARGET  is the desired network output
BETA    is the learning rate for the LMS algorithm and should be in the
        range 0 to 2.
ADDRS   is a C-dimensional vector of addresses returned by the function
IP      is an input vector
IPRANGE is the range in which each element of IP is expected to fall, i.e.
        you must scale IP(i) to fall in the range 0 to IPRANGE for all i
C       is the number of addresses returned
WIDTH   is related to the extent of local generalisation
MEMSIZE is the total number of weights used by the network. It is the
        range in which the returned addresses will fall and therefore the
        size of matrix required to store the network weights. Setting
        MEMSIZE to 0 disables the hash coding of addresses that otherwise
        happens within the function ADDRCMAC otherwise. With hashing
        disabled, the range of addresses returned is given by
        C*(IPRANGE/WIDTH)^IPDIM where IPDIM is the number of elements in
        the input IP. Other than for low dimensional inputs, the range of
        addresses returned will become unmanageably large - that's why hash
        coding is the default. However, hash coding introduces extra noise
        into the network output and for low dimensional problems, it may be
        useful to disable it.
*/

CMAC::CMAC(size_t IpSize, size_t OpSize, int c, int Qlevels, long MemSize=0)
    :c(c),beta(0.4),Qlevels(Qlevels),IpSize(IpSize),OpSize(OpSize)
{
    /* compute network parameters */
    Qwidth = 1.0/Qlevels;     // 单层量化等级，memreq = c*ceil(iprange/width);
    offset= Qwidth/c;         // 层间间隔 dq
    addrs = new long[c];

    // 直接编码或哈希编码（哈希编码可以指定占用内存的大小，但会引入噪声）
    use_hash= (MemSize!=0);
    memsize = use_hash? MemSize  :  c*pow(Qlevels,(int)IpSize);

    wts = new double*[OpSize];
    for (size_t i=0;i<OpSize;i++)
    {
        wts[i]=new double[memsize];
        memset(wts[i],0,memsize*sizeof(double));
    }

    IpRange = new double[OpSize];
    Imax    = new double[IpSize];
    Imin    = new double[IpSize];

}

void CMAC::SetInputRange(double *imin, double *imax)
{
    for(size_t i=0;i<IpSize;i++)
    {
        assert(imin[i]<imax[i]);
        Imin[i] = imin[i];
        Imax[i] = imax[i];
        IpRange[i]=Imax[i]-Imin[i];
    }
}

CMAC::~CMAC()
{
    //SaveTable("wtable.dat");

    delete [] addrs,Imin,Imax,IpRange;

    for (size_t i=0;i<OpSize;i++)
        delete [] wts[i];

    //delete [] wts;
}


#define HASH 12345

void CMAC::Calc(double ip[], double op[])
{
    double shift, address, ofs=0.0;

    // 输入归一化
    for(size_t i=0;i<IpSize;i++)
    {
        assert( Imin[i]<=ip[i] && ip[i]<=Imax[i] );
        ip[i]=(ip[i]-Imin[i])/IpRange[i];
    }
    // 输出初始化
    memset(op,0,OpSize*sizeof(double));

    /* compute addresses */
    for (size_t i=0 ; i<c ; i++)
    {
        address = 0;
        shift = 1.0;
        for (size_t j=0 ; j<IpSize ; j++)
        {
            address += (((int)((ip[j] + ofs)/Qwidth)) % Qlevels)*shift;
            shift *= Qlevels;
        }
        address += shift*i;

        /* check for hashing */
        if (use_hash)
            addrs[i] = (long)(log(address+1)*HASH) % memsize;
        else
            addrs[i] = (long)(address);

        ofs += offset;

        for (size_t o=0; o<OpSize; o++)
            op[o] += wts[o][addrs[i]];
    }
}


void CMAC::Train(double delta[])
{
    for (size_t o=0; o<OpSize; o++)
    {
        delta[o]=delta[o]/(double)c;
        for (size_t i=0; i<c ; i++)
            wts[o][addrs[i]] += delta[o];
    }

}

bool CMAC::LoadTable(std::string filename)
{
    std::ifstream data(filename);
    if(!data)
        return false;
    double i,o;

    data>>i>>o;

    if(i!=IpSize || o!= OpSize)
    {
        data.close();
        return false;
    }

    for(size_t i=0;i<OpSize;i++)
        for(size_t j=0;j<memsize;j++)
        {
            data>>wts[i][j];
        }

    data.close();
    return true;
}

void CMAC::SaveTable(std::string filename)
{
    std::ofstream data(filename);

    data<<IpSize<<' '<<OpSize<<std::endl;
    for(size_t i=0;i<OpSize;i++)
    {
        for(size_t j=0;j<memsize;j++)
        {
            data<<wts[i][j]<<'\t';
        }
        data<<std::endl;
    }
    data.flush();
    data.close();
}
