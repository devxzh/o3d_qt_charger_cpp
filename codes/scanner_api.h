#ifndef __SCANNER_API_H__
#define __SCANNER_API_H__

#ifdef EXPORT_DLL
#	define DLL_EXPORT __declspec(dllexport)
#else
#	define DLL_EXPORT __declspec(dllimport)
#endif

typedef enum 
{
    DISCOVERY_TYPE_NOTIFY, 
} DISCOVERY_TYPE_E;
	
typedef int (*DiscoveryCallback)(DISCOVERY_TYPE_E type, char *dataAddr, int dataSize);

_declspec(dllexport) int Discovery_Start(DiscoveryCallback callback);
_declspec(dllexport) int Discovery_Stop();
_declspec(dllexport) int Discovery_ConfigIp(const char *mac, const char *ip, const char *netmask);

typedef void (*ConnectCallback)(bool connected);

typedef enum 
{
    STREAM_TYPE_2D_PIC,  
    STREAM_TYPE_POINT_CLOUD,
	STREAM_TYPE_POINT_DEPTH, 
    STREAM_TYPE_POINT_XY,
    STREAM_TYPE_SCAN_OVER,
    STREAM_TYPE_SCAN_ERROR,
} STREAM_TYPE_E;

typedef int (*StreamCallback)(STREAM_TYPE_E type, char *dataAddr, int dataSize);

typedef enum 
{ 
	SCANNER_REQUEST_START_CALIB = 0x100, 
	SCANNER_REQUEST_STOP_CALIB, 
	SCANNER_REQUEST_GET_FRAME, 
	SCANNER_REQUEST_CALIBRATE,
	SCANNER_REQUEST_START_SCAN,
	SCANNER_REQUEST_STOP_SCAN,
	SCANNER_REQUEST_GET_PARAM,
	SCANNER_REQUEST_SET_PARAM,
	SCANNER_REQUEST_CLOSE_CONNECT,
	SCANNER_REQUEST_PREPARE_FIRMWARE,
	SCANNER_REQUEST_START_CALIB_SYSTEM,
	SCANNER_REQUEST_SET_POINTCLOUD_OPTION,
	SCANNER_REQUEST_GET_POINTCLOUD_OPTION,
	SCANNER_REQUEST_SET_PARAM_EX,
	SCANNER_REQUEST_GET_PARAM_EX,
	SCANNER_REQUEST_SET_POINTCLOUD_OPTION_EX,
	SCANNER_REQUEST_GET_POINTCLOUD_OPTION_EX,
	SCANNER_REQUEST_RESTORE_DEFAULT,
	SCANNER_REQUEST_GET_COREDUMP_FILE,
	SCANNER_REQUEST_GET_LOG_FILES,
	SCANNER_REQUEST_GET_DEVICE_ID,
	SCANNER_REQUEST_GET_CALIB_PARAM,
	SCANNER_REQUEST_GET_FULLSIZE_FRAME,
	SCANNER_REQUEST_SAVE_SCAN_PARAM,
	SCANNER_REQUEST_SET_DEVICE_ID,
	SCANNER_REQUEST_SET_WORK_DISTANCE,
	SCANNER_REQUEST_GET_WORK_DISTANCE,
	SCANNER_REQUEST_BUFF
}SCANNER_REQUEST_CMD_E;

typedef enum 
{
	SCANNER_RESULT_START_CALIB = 0x200, 
	SCANNER_RESULT_GET_FRAME,
	SCANNER_RESULT_CALIBRATE,	
	SCANNER_RESULT_GET_PARAM,
	SCANNER_RESULT_SCAN_ERROR,
	SCANNER_RESULT_SCAN_2D_PIC,
	SCANNER_RESULT_SCAN_POINT_CLOUD,
	SCANNER_RESULT_SCAN_POINT_DEPTH,
	SCANNER_RESULT_SCAN_POINT_XY,
	SCANNER_RESULT_SCAN_OVER,
	SCANNER_RESULT_START_CALIB_SYSTEM,
	SCANNER_RESULT_GET_OPTION,
	SCANNER_RESULT_GET_PARAM_EX,
	SCANNER_RESULT_GET_OPTION_EX,
	SCANNER_RESULT_COREDUMP_FILE,
	SCANNER_RESULT_LOG_FILES,
	SCANNER_RESULT_GET_DEVICE_ID,
	SCANNER_RESULT_GET_CALIB_PARAM,
	SCANNER_RESULT_GET_FULLSIZE_FRAME,
	SCANNER_RESULT_GET_WORK_DISTANCE,
	SCANNER_RESULT_BUFF
}SCANNER_RESULT_CMD_E;


typedef struct _scanner_frame_s
{
    int width;
    int height;
	int pitch;
	int depth;
	char *pdata;
	int datalen;
}SCANNER_FRAME_S;

typedef struct _scanner_calib_result_s
{
    int curIndex;
	int totalNum;
	bool curResult;
	bool finalResult;
	double errValue;
}SCANNER_CALIB_RESULT_S;

typedef struct _scanner_param_s
{
	unsigned int gain;
	unsigned int dlp_red;
	unsigned int dlp_green;
	unsigned int dlp_blue;
}SCANNER_PARAM_S;

typedef struct _scanner_option_s
{
    bool point_cloud_xyz_disable;
	bool point_cloud_depth_disable;
	bool point_cloud_H_V;
	bool point_cloud_HDR;
	unsigned int point_cloud_max_gain_HDR;
	bool point_cloud_smooth;
}SCANNER_OPTION_S;


typedef struct _scanner_param_ext_s
{
    unsigned int res_w;
	unsigned int res_h;
	unsigned int frame_rate;
	unsigned int expose_time_ms;
	unsigned int proj_expose_time;
    unsigned int proj_period_time;
}SCANNER_PARAM_EXT_S;


typedef struct _scanner_option_ext_s
{
	bool point_cloud_save_scanimage;
	bool point_cloud_continue_scan;
}SCANNER_OPTION_EXT_S;

typedef struct _device_info{ 
    char   mac[64]; 
    char   ip[64];  
    char   status[64];
    char   sdkversion[64];
    char   deviceid[64];
}DEVICE_INFO;

typedef struct _scanner_calib_param_s{ 
    double   intrinsic[9]; 
    double   distortion[5];  
}SCANNER_CALIB_PARAM_S;

typedef struct _scanner_workdistance_config_s
{
    int workdistance_min;
    int workdistance_max;
}SCANNER_WORKDISTANCE_CONFIG_S;



class  Scanner
{
public:
	virtual ~Scanner(){}

	virtual int Open(const char *scannerUri, ConnectCallback callback) = 0;
	virtual int Close() = 0;	

	virtual int StartCalibrate() = 0;
	virtual int StartCalibrateSystem() = 0;
	virtual int StopCalibrate() = 0;

	virtual int GetPreviewFrame(SCANNER_FRAME_S *pstCalibFrame) = 0;
	virtual int Calibrate(SCANNER_CALIB_RESULT_S *pstCalibResult) = 0;	

	virtual int Get2DPicture(SCANNER_FRAME_S *pstFrame) = 0;
	virtual int StartScan(StreamCallback callback) = 0;
	virtual int StopScan() = 0;

	
	virtual int SetScanParam(SCANNER_PARAM_S *pstParam) = 0;
	virtual int GetScanParam(SCANNER_PARAM_S *pstParam) = 0;
	virtual int SetPointCloudOption(SCANNER_OPTION_S *pstOption) = 0;
	virtual int GetPointCloudOption(SCANNER_OPTION_S *pstOption) = 0;	

	virtual int SetScanParamEx(SCANNER_PARAM_EXT_S *pstParam) = 0;
	virtual int GetScanParamEx(SCANNER_PARAM_EXT_S *pstParam) = 0;
	virtual int SetPointCloudOptionEx(SCANNER_OPTION_EXT_S *pstOption) = 0;
	virtual int GetPointCloudOptionEx(SCANNER_OPTION_EXT_S *pstOption) = 0;
    virtual int GetWorkDistance(SCANNER_WORKDISTANCE_CONFIG_S *pstWordDistance) = 0;
	
	virtual int RestoreDefault() = 0;
	
	virtual int PrepareProjectFirmware() = 0;	
	virtual int GetCoreDumpFile(const char * filePath) = 0;	
	virtual bool IsCoreDumpFileUploading() = 0;
	virtual int GetLogFiles(const char * filePath) = 0;
    virtual int GetDeviceId(char * pstParam) = 0;
    virtual int GetClientVersion(char * pstParam) = 0;
    virtual int GetCalibrateParam(SCANNER_CALIB_PARAM_S *pstCalibParm) = 0;
	virtual int SaveScanParam() = 0;
};


_declspec(dllexport) Scanner * CreateScannerImp();

#endif

