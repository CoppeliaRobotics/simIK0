#include "simIK0.h"
#include "envCont.h"
#include <simLib/simLib.h>
#include <ik.h>
#include <simMath/4X4Matrix.h>
#include <simMath/mathFuncs.h>
#include <iostream>
#include <cstdio>
#include <algorithm>

#ifdef _WIN32
#ifdef QT_COMPIL
    #include <direct.h>
#else
    #include <shlwapi.h>
    #pragma comment(lib, "Shlwapi.lib")
#endif
#endif
#ifdef _WIN32
    #include <Windows.h>
    typedef CRITICAL_SECTION WMutex;
#endif
#if defined (__linux) || defined (__APPLE__)
    #include <unistd.h>
    #include <pthread.h>
    typedef pthread_mutex_t WMutex;
#endif

static LIBRARY simLib;
static CEnvCont* _allEnvironments;
static std::string _pluginName;

struct SJointDependCB
{
    int ikEnv;
    int ikSlave;
    std::string cbString;
    int scriptHandle;
};

static std::vector<SJointDependCB> jointDependInfo;

void _removeJointDependencyCallback(int envId,int slaveJoint)
{
    for (int i=0;i<int(jointDependInfo.size());i++)
    {
        if (jointDependInfo[i].ikEnv==envId)
        {
            if ( (jointDependInfo[i].ikSlave==slaveJoint)||(slaveJoint==-1) )
            {
                jointDependInfo.erase(jointDependInfo.begin()+i);
                i--;
            }
        }
    }
}

SIM_DLLEXPORT int simInit(const char* pluginName)
{
    _pluginName=pluginName;
    char curDirAndFile[1024];
#ifdef _WIN32
    #ifdef QT_COMPIL
        _getcwd(curDirAndFile, sizeof(curDirAndFile));
    #else
        GetModuleFileName(NULL,curDirAndFile,1023);
        PathRemoveFileSpec(curDirAndFile);
    #endif
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);

#ifdef _WIN32
    temp+="\\coppeliaSim.dll";
#elif defined (__linux)
    temp+="/libcoppeliaSim.so";
#elif defined (__APPLE__)
    temp+="/libcoppeliaSim.dylib";
#endif /* __linux || __APPLE__ */

    simLib=loadSimLibrary(temp.c_str());
    if (simLib==NULL)
    {
        simAddLog(pluginName,sim_verbosity_errors,"could not find or correctly load the CoppeliaSim library. Cannot start the plugin.");
        return(0);
    }
    if (getSimProcAddresses(simLib)==0)
    {
        simAddLog(pluginName,sim_verbosity_errors,"could not find all required functions in the CoppeliaSim library. Cannot start the plugin.");
        unloadSimLibrary(simLib);
        return(0);
    }

    return(3); // 3 since V4.6.0
}

SIM_DLLEXPORT void simCleanup()
{
}

SIM_DLLEXPORT void simMsg(int message,int* auxData,void* auxPointer)
{
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------

SIM_DLLEXPORT int ikPlugin_createEnv()
{
    int retVal=-1;
    ikCreateEnvironment(&retVal,1|2); // protected (1) and backward compatible for old GUI-IK(2)
    return(retVal);
}

SIM_DLLEXPORT void ikPlugin_eraseEnvironment(int ikEnv)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikEraseEnvironment();
}

SIM_DLLEXPORT void ikPlugin_eraseObject(int ikEnv,int objectHandle)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikEraseObject(objectHandle);
}

SIM_DLLEXPORT void ikPlugin_setObjectParent(int ikEnv,int objectHandle,int parentObjectHandle)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetObjectParent(objectHandle,parentObjectHandle,false);
}

SIM_DLLEXPORT int ikPlugin_createDummy(int ikEnv)
{
    int retVal=-1;
    if (ikSwitchEnvironment(ikEnv,true))
        ikCreateDummy(nullptr,&retVal);
    return(retVal);
}

SIM_DLLEXPORT void ikPlugin_setLinkedDummy(int ikEnv,int dummyHandle,int linkedDummyHandle)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetLinkedDummy(dummyHandle,linkedDummyHandle);
}

SIM_DLLEXPORT int ikPlugin_createJoint(int ikEnv,int jointType)
{
    int retVal=-1;
    if (ikSwitchEnvironment(ikEnv,true))
        ikCreateJoint(nullptr,jointType,&retVal);
    return(retVal);
}

SIM_DLLEXPORT void ikPlugin_setJointMode(int ikEnv,int jointHandle,int jointMode)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetJointMode(jointHandle,jointMode);
}

SIM_DLLEXPORT void ikPlugin_setJointInterval(int ikEnv,int jointHandle,bool cyclic,const double* intervalMinAndRange)
{
    if (ikSwitchEnvironment(ikEnv,true))
    {
#ifdef switchToDouble
        ikSetJointInterval(jointHandle,cyclic,intervalMinAndRange);
#else
        double v[2]={double(intervalMinAndRange[0]),double(intervalMinAndRange[1])};
        ikSetJointInterval(jointHandle,cyclic,v);
#endif
    }
}

SIM_DLLEXPORT void ikPlugin_setJointScrewPitch(int ikEnv,int jointHandle,double pitch)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetJointScrewPitch(jointHandle,pitch);
}

SIM_DLLEXPORT void ikPlugin_setJointIkWeight(int ikEnv,int jointHandle,double ikWeight)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetJointWeight(jointHandle,ikWeight);
}

SIM_DLLEXPORT void ikPlugin_setJointMaxStepSize(int ikEnv,int jointHandle,double maxStepSize)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetJointMaxStepSize(jointHandle,maxStepSize);
}

SIM_DLLEXPORT void ikPlugin_setJointDependency(int ikEnv,int jointHandle,int dependencyJointHandle,double offset,double mult)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetJointDependency(jointHandle,dependencyJointHandle,offset,mult);
}

SIM_DLLEXPORT double ikPlugin_getJointPosition(int ikEnv,int jointHandle)
{
    double p=0.0;
    if (ikSwitchEnvironment(ikEnv,true))
        ikGetJointPosition(jointHandle,&p);
    return(p);
}

SIM_DLLEXPORT void ikPlugin_setJointPosition(int ikEnv,int jointHandle,double position)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetJointPosition(jointHandle,position);
}

SIM_DLLEXPORT void ikPlugin_getSphericalJointQuaternion(int ikEnv,int jointHandle,double quaternion[4])
{
    C7Vector tr;
    tr.setIdentity();
    if (ikSwitchEnvironment(ikEnv,true))
        ikGetJointTransformation(jointHandle,&tr);
    quaternion[0]=tr.Q(0);
    quaternion[1]=tr.Q(1);
    quaternion[2]=tr.Q(2);
    quaternion[3]=tr.Q(3);
}

SIM_DLLEXPORT void ikPlugin_setSphericalJointQuaternion(int ikEnv,int jointHandle,const double* quaternion)
{
    C4Vector q;
    q(0)=quaternion[0];
    q(1)=quaternion[1];
    q(2)=quaternion[2];
    q(3)=quaternion[3];
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetSphericalJointQuaternion(jointHandle,&q);
}

SIM_DLLEXPORT int ikPlugin_createIkGroup(int ikEnv)
{
    int retVal=-1;
    if (ikSwitchEnvironment(ikEnv,true))
        ikCreateGroup(nullptr,&retVal);
    return(retVal);
}

SIM_DLLEXPORT void ikPlugin_eraseIkGroup(int ikEnv,int ikGroupHandle)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikEraseGroup(ikGroupHandle);
}

SIM_DLLEXPORT void ikPlugin_setIkGroupFlags(int ikEnv,int ikGroupHandle,int flags)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetGroupFlags(ikGroupHandle,flags);
}

SIM_DLLEXPORT void ikPlugin_setIkGroupCalculation(int ikEnv,int ikGroupHandle,int method,double damping,int maxIterations)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetGroupCalculation(ikGroupHandle,method,damping,maxIterations);
}

SIM_DLLEXPORT int ikPlugin_addIkElement(int ikEnv,int ikGroupHandle,int tipHandle)
{
    int retVal=-1;
    if (ikSwitchEnvironment(ikEnv,true))
        ikAddElement(ikGroupHandle,tipHandle,&retVal);
    return(retVal);
}

SIM_DLLEXPORT void ikPlugin_eraseIkElement(int ikEnv,int ikGroupHandle,int ikElementHandle)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikEraseElement(ikGroupHandle,ikElementHandle);
}

SIM_DLLEXPORT void ikPlugin_setIkElementFlags(int ikEnv,int ikGroupHandle,int ikElementHandle,int flags)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetElementFlags(ikGroupHandle,ikElementHandle,flags);
}

SIM_DLLEXPORT void ikPlugin_setIkElementBase(int ikEnv,int ikGroupHandle,int ikElementHandle,int baseHandle,int constraintsBaseHandle)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetElementBase(ikGroupHandle,ikElementHandle,baseHandle,constraintsBaseHandle);
}

SIM_DLLEXPORT void ikPlugin_setIkElementConstraints(int ikEnv,int ikGroupHandle,int ikElementHandle,int constraints)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetElementConstraints(ikGroupHandle,ikElementHandle,constraints);
}

SIM_DLLEXPORT void ikPlugin_setIkElementPrecision(int ikEnv,int ikGroupHandle,int ikElementHandle,double linearPrecision,double angularPrecision)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetElementPrecision(ikGroupHandle,ikElementHandle,linearPrecision,angularPrecision);
}

SIM_DLLEXPORT void ikPlugin_setIkElementWeights(int ikEnv,int ikGroupHandle,int ikElementHandle,double linearWeight,double angularWeight)
{
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetElementWeights(ikGroupHandle,ikElementHandle,linearWeight,angularWeight,1.0);
}

SIM_DLLEXPORT int ikPlugin_handleIkGroup(int ikEnv,int ikGroupHandle)
{
    int retVal=-1;
    if (ikSwitchEnvironment(ikEnv,true))
    {
        std::vector<int> gr;
        gr.push_back(ikGroupHandle);
        ikHandleGroups(&gr,&retVal,nullptr);
        if ( (retVal&ik_calc_notperformed)!=0 )
            retVal=0; // ik_result_not_performed
        else if ( (retVal&(ik_calc_cannotinvert|ik_calc_notwithintolerance))!=0 )
            retVal=2; // previously ik_result_fail
        else retVal=1; // previously ik_result_success
    }
    return(retVal);
}

SIM_DLLEXPORT bool ikPlugin_computeJacobian(int ikEnv,int ikGroupHandle,int options)
{
    bool retVal=false;
    if (ikSwitchEnvironment(ikEnv,true))
        ikComputeJacobian_old(ikGroupHandle,options,&retVal);
    return(retVal);
}

SIM_DLLEXPORT double* ikPlugin_getJacobian(int ikEnv,int ikGroupHandle,int* matrixSize)
{
    double* retVal=nullptr;
    if (ikSwitchEnvironment(ikEnv,true))
    {
        size_t ms[2];
        double* m=ikGetJacobian_old(ikGroupHandle,ms);
        if (m!=nullptr)
        {
            matrixSize[0]=int(ms[0]);
            matrixSize[1]=int(ms[1]);
            retVal=reinterpret_cast<double*>(simCreateBuffer(int(sizeof(double)*ms[0]*ms[1])));
            for (size_t i=0;i<ms[0]*ms[1];i++)
                retVal[i]=m[i];
            ikReleaseBuffer(m);
        }
    }
    return(retVal);
}

SIM_DLLEXPORT double ikPlugin_getManipulability(int ikEnv,int ikGroupHandle)
{
    double retVal=0.0;
    if (ikSwitchEnvironment(ikEnv,true))
        ikGetManipulability_old(ikGroupHandle,&retVal);
    return(retVal);
}

SIM_DLLEXPORT void ikPlugin_getObjectLocalTransformation(int ikEnv,int objectHandle,double* pos,double* quat)
{
    C7Vector tr;
    tr.setIdentity();
    if (ikSwitchEnvironment(ikEnv,true))
        ikGetObjectTransformation(objectHandle,ik_handle_parent,&tr);
    pos[0]=tr.X(0);
    pos[1]=tr.X(1);
    pos[2]=tr.X(2);
    quat[0]=tr.Q(0);
    quat[1]=tr.Q(1);
    quat[2]=tr.Q(2);
    quat[3]=tr.Q(3);
}

SIM_DLLEXPORT void ikPlugin_setObjectLocalTransformation(int ikEnv,int objectHandle,const double* pos,const double* quat)
{
    C7Vector tr;
    tr.X(0)=pos[0];
    tr.X(1)=pos[1];
    tr.X(2)=pos[2];
    tr.Q(0)=quat[0];
    tr.Q(1)=quat[1];
    tr.Q(2)=quat[2];
    tr.Q(3)=quat[3];
    if (ikSwitchEnvironment(ikEnv,true))
        ikSetObjectTransformation(objectHandle,ik_handle_parent,&tr);
}

static size_t _validationCallback_jointCnt;
static bool(*__validationCallback)(double*);
static double* _validationCallback_config;

bool _validationCallback(double* conf)
{
    for (size_t i=0;i<_validationCallback_jointCnt;i++)
        _validationCallback_config[i]=conf[i];
    return(__validationCallback(_validationCallback_config));
}

SIM_DLLEXPORT char* ikPlugin_getConfigForTipPose(int ikEnv,int ikGroupHandle,int jointCnt,const int* jointHandles,double thresholdDist,int maxIterations,int* result,double* retConfig,const double* metric,bool(*validationCallback)(double*),const int* jointOptions,const double* lowLimits,const double* ranges)
{
    char* retVal=nullptr;
    if (ikSwitchEnvironment(ikEnv,true))
    {
        std::vector<double> _retConfig;
        _retConfig.resize(size_t(jointCnt));
        double* _metric=nullptr;
        double __metric[4];
        if (metric!=nullptr)
        {
            __metric[0]=metric[0];
            __metric[1]=metric[1];
            __metric[2]=metric[2];
            __metric[3]=metric[3];
            _metric=__metric;
        }
        double* _lowLimits=nullptr;
        std::vector<double> __lowLimits;
        if (lowLimits!=nullptr)
        {
            for (size_t i=0;i<size_t(jointCnt);i++)
                __lowLimits.push_back(lowLimits[i]);
            _lowLimits=&__lowLimits[0];
        }
        double* _ranges=nullptr;
        std::vector<double> __ranges;
        if (ranges!=nullptr)
        {
            for (size_t i=0;i<size_t(jointCnt);i++)
                __ranges.push_back(ranges[i]);
            _ranges=&__ranges[0];
        }
        _validationCallback_jointCnt=size_t(jointCnt);
        std::vector<double> __valCb_j;
        __valCb_j.resize(size_t(jointCnt));
        _validationCallback_config=&__valCb_j[0];
        __validationCallback=validationCallback;
        bool(*_validationCb)(double*)=nullptr;
        if (validationCallback!=nullptr)
            _validationCb=_validationCallback;
        result[0]=ikGetConfigForTipPose(ikGroupHandle,size_t(jointCnt),jointHandles,thresholdDist,maxIterations,&_retConfig[0],_metric,_validationCb,jointOptions,_lowLimits,_ranges);
        if (result[0]>0)
        {
            for (size_t i=0;i<_retConfig.size();i++)
                retConfig[i]=_retConfig[i];
        }
        if (result[0]<0)
        {
            std::string err(ikGetLastError());
            retVal=(char*)simCreateBuffer(int(err.size()+1));
            for (size_t i=0;i<err.size();i++)
                retVal[i]=err[i];
            retVal[err.size()]=0;
        }
    }
    return(retVal);
}
