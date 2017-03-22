#include <string>

#include  "stdlib.h"
#include  "stdio.h"

#include "Karto.h"
#include "Macros.h"
#include "Math.h"
#include "Types.h"
#include "Mapper.h"
#include "g2o_solver.h"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <opencv2/opencv.hpp>


using namespace karto;
using namespace std;
using namespace cv;

/**
* Sample code to demonstrate karto map creation
* Create a laser range finder device and three "dummy" range scans.
* Add the device and range scans to a karto Mapper.

karto::Dataset* CreateMap(karto::Mapper* pMapper)
{
	karto::Dataset* pDataset = new karto::Dataset();

	/////////////////////////////////////
	// Create a laser range finder device - use SmartPointer to let karto subsystem manage memory.
	karto::Name name("laser0");
	karto::LaserRangeFinder* pLaserRangeFinder = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, name);
    pLaserRangeFinder->SetOffsetPose(karto::Pose2(0.0, 0.0, 0.0));
	pLaserRangeFinder->SetAngularResolution(karto::math::DegreesToRadians(0.5));
	pLaserRangeFinder->SetRangeThreshold(12.0);

	pDataset->Add(pLaserRangeFinder);

	/////////////////////////////////////
	// Create three localized range scans, all using the same range readings, but with different poses. 
	karto::LocalizedRangeScan* pLocalizedRangeScan = NULL;

	// 1. localized range scan

	// Create a vector of range readings. Simple example where all the measurements are the same value.
	std::vector<kt_double> readings;
	for (int i = 0; i<361; i++)
	{
		readings.push_back(3.0);
	}

	// create localized range scan
	pLocalizedRangeScan = new karto::LocalizedRangeScan(name, readings);
	pLocalizedRangeScan->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
	pLocalizedRangeScan->SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));

	// Add the localized range scan to the mapper
	pMapper->Process(pLocalizedRangeScan);
	std::cout << "Pose: " << pLocalizedRangeScan->GetOdometricPose() << " Corrected Pose: " << pLocalizedRangeScan->GetCorrectedPose() << std::endl;

	// Add the localized range scan to the dataset
	pDataset->Add(pLocalizedRangeScan);

	// 2. localized range scan

	// create localized range scan
	pLocalizedRangeScan = new karto::LocalizedRangeScan(name, readings);
	pLocalizedRangeScan->SetOdometricPose(karto::Pose2(1.0, 0.0, 1.57));
	pLocalizedRangeScan->SetCorrectedPose(karto::Pose2(1.0, 0.0, 1.57));

	// Add the localized range scan to the mapper
	pMapper->Process(pLocalizedRangeScan);
	std::cout << "Pose: " << pLocalizedRangeScan->GetOdometricPose() << " Corrected Pose: " << pLocalizedRangeScan->GetCorrectedPose() << std::endl;

	// Add the localized range scan to the dataset
	pDataset->Add(pLocalizedRangeScan);

	// 3. localized range scan

	// create localized range scan
	pLocalizedRangeScan = new karto::LocalizedRangeScan(name, readings);
	pLocalizedRangeScan->SetOdometricPose(karto::Pose2(1.0, -1.0, 2.35619449));
	pLocalizedRangeScan->SetCorrectedPose(karto::Pose2(1.0, -1.0, 2.35619449));

	// Add the localized range scan to the mapper
	pMapper->Process(pLocalizedRangeScan);
	std::cout << "Pose: " << pLocalizedRangeScan->GetOdometricPose() << " Corrected Pose: " << pLocalizedRangeScan->GetCorrectedPose() << std::endl;

	// Add the localized range scan to the dataset
	pDataset->Add(pLocalizedRangeScan);

	return pDataset;
}
*/

/**
* Sample code to demonstrate basic occupancy grid creation and print occupancy grid.
*/
karto::OccupancyGrid* CreateOccupancyGrid(karto::Mapper* pMapper, kt_double resolution)
{
    //std::cout << "Generating map..." << std::endl;

	// Create a map (occupancy grid) - time it
	/* pMapper->GetAllProcessedScans() 获取所有的激光扫描线 */
	karto::OccupancyGrid* pOccupancyGrid = karto::OccupancyGrid::CreateFromScans(pMapper->GetAllProcessedScans(), resolution);

	return pOccupancyGrid;
}

/**
* Sample code to print a basic occupancy grid

void PrintOccupancyGrid(karto::OccupancyGrid* pOccupancyGrid)
{
	if (pOccupancyGrid != NULL)
	{
		// Output ASCII representation of map
		kt_int32s width = pOccupancyGrid->GetWidth();
		kt_int32s height = pOccupancyGrid->GetHeight();
		karto::Vector2<kt_double> offset = pOccupancyGrid->GetCoordinateConverter()->GetOffset();

		std::cout << "width = " << width << ", height = " << height << ", scale = " << pOccupancyGrid->GetCoordinateConverter()->GetScale() << ", offset: " << offset.GetX() << ", " << offset.GetY() << std::endl;
		for (kt_int32s y = height - 1; y >= 0; y--)
		{
			for (kt_int32s x = 0; x<width; x++)
			{
				// Getting the value at position x,y
				kt_int8u value = pOccupancyGrid->GetValue(karto::Vector2<kt_int32s>(x, y));

				switch (value)
				{
				case karto::GridStates_Unknown:
					std::cout << "*";
					break;
				case karto::GridStates_Occupied:
					std::cout << "X";
					break;
				case karto::GridStates_Free:
					std::cout << " ";
					break;
				default:
					std::cout << "?";
				}
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
	}
}
*/

int main(int argc, char** argv)
{
    if(argc != 4)
    {
        cout << "Usage: ./main device data_path isUseScanMatch" << endl;
        return 0;
    }

    ifstream infile;
    string fileName = argv[2];
    infile.open(fileName.data());
    assert(infile.is_open());

    vector<string> fn;
    boost::split(fn, fileName, boost::is_any_of("/"));
    vector<string> fn2;
    boost::split(fn2, fn[fn.size()-1], boost::is_any_of("."));
    ofstream outfile;
    outfile.open("../log/" + fn2[0] + ".log");
    assert(outfile.is_open());

	karto::Mapper* pMapper = new karto::Mapper();

	if (pMapper != NULL)
	{
		std::cout << "----------------" << std::endl << std::endl;
	}

	pMapper->Reset();

    G2OSolver* pSolver = new G2OSolver();
    pMapper->SetScanSolver(pSolver);

    LaserRangeFinderType device;
    int device_num = atoi(const_cast<const char *>(argv[1]));
    switch(device_num)
    {
    case 0 :
        device = karto::LaserRangeFinder_Custom;
        break;
    case 1 :
        device = karto::LaserRangeFinder_Sick_LMS100;
        break;
    case 2 :
        device = karto::LaserRangeFinder_Sick_LMS200;
        break;
    case 3 :
        device = karto::LaserRangeFinder_Sick_LMS291;
        break;
    case 4 :
        device = karto::LaserRangeFinder_Hokuyo_UTM_30LX;
        break;
    case 5 :
        device = karto::LaserRangeFinder_Hokuyo_URG_04LX;
        break;
    case 6 :
        device = karto::LaserRangeFinder_Leo;
        break;
    default :
        device = karto::LaserRangeFinder_Custom;
        break;
    }

    if(string(argv[3]) == "false")
    {
        pMapper->SetUseScanMatching(false);

    }
    pMapper->setParamDoLoopClosing(false);

	karto::Dataset* pDataset = new karto::Dataset();
	karto::Name name("laser0");
    karto::LaserRangeFinder* pLaserRangeFinder = karto::LaserRangeFinder::CreateLaserRangeFinder(device, name); // 根据不同设备设置了不同的角度分辨率等参数

    double leo_scale = 100.0;

    // 设置阈值
    if(device_num == 6)
    {
        pLaserRangeFinder->SetRangeThreshold(12.0);
        pLaserRangeFinder->SetOffsetPose(karto::Pose2(93.3/leo_scale, 93.3/leo_scale, 3.14159265/4));             /* 激光器相对于机器人中心的坐标 */
    }
    else
    {
        pLaserRangeFinder->SetRangeThreshold(12.0);
        pLaserRangeFinder->SetOffsetPose(karto::Pose2(0.0, 0.0, 0.0));             /* 激光器相对于机器人中心的坐标 */
    }

	pDataset->Add(pLaserRangeFinder);
    double occuGridResolution = 0.1;

    string s;
    int nCount = 0;
    bool step = true;
	while (getline(infile, s))
	{
        //printf("nCount = %d\n", nCount);
		vector<string> data;
		boost::split(data, s, boost::is_any_of(" "));

        int beam_num = atoi(const_cast<const char *>(data.at(1).c_str()));

        double ranges_double[beam_num+6];

        karto::LocalizedRangeScan* pLocalizedRangeScan = NULL;
		
		std::vector<kt_double> readings;
		
        for (int i = 0; i < beam_num; i++)
		{
            if(device_num != 6)
            {
                ranges_double[i] = atof(const_cast<const char *>(data.at(i + 2).c_str()));
                readings.push_back(ranges_double[i]);
            }
            else
            {
                ranges_double[i] = atof(const_cast<const char *>(data.at(beam_num - i - 1 + 2).c_str())) / leo_scale;

                if(i < 50 || i > 180)
                {
                      ranges_double[i] = 0;
                }

                readings.push_back(ranges_double[i]);
            }

		}

        // beams of LaserRangeFinder_Custom is 181
        if(beam_num == 180)
            readings.push_back(ranges_double[179]);

		float pos[3];
		for (int i = 0; i < 3; i++)
		{
            if(device_num != 6)
                pos[i] = atof(const_cast<const char *>(data.at(i + beam_num+2).c_str()));
            else
            {
                if(i != 2)
                    pos[i] = atof(const_cast<const char *>(data.at(i + beam_num+2).c_str())) / leo_scale;
                else
                    pos[i] = atof(const_cast<const char *>(data.at(i + beam_num+2).c_str()));
            }
		}
        //cout << pos[0] << " " << pos[1] << " " << pos[2] << endl;

		pLocalizedRangeScan = new karto::LocalizedRangeScan(name, readings);
		pLocalizedRangeScan->SetOdometricPose(karto::Pose2(pos[0], pos[1], pos[2]));
		pLocalizedRangeScan->SetCorrectedPose(karto::Pose2(pos[0], pos[1], pos[2]));

        kt_bool precess_state = pMapper->Process(pLocalizedRangeScan);
		pDataset->Add(pLocalizedRangeScan);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //ComputePositionalCovariance : -1.87374 0.15 0.02 0.0349066
        //ComputeAngularCovariance : -1.89374 0.0174533 0.00349066

        cout << "ID : " << pLocalizedRangeScan->GetStateId() << endl;

        Matrix3 covariance;
        covariance.SetToIdentity();

        if(string(argv[3]) == "false" && precess_state)
        {
            pMapper->GetGraph()->AddVertex(pLocalizedRangeScan);
            Pose2 currpose = pLocalizedRangeScan->GetSensorPose();

            if(nCount != 0)
            {
                karto::LocalizedRangeScan* pTempScan = pMapper->GetMapperSensorManager()->GetScan(pLocalizedRangeScan->GetStateId() - 1);
                Pose2 lastpose = pTempScan->GetSensorPose();

                double diffX = (lastpose.GetX() - currpose.GetX());
                double diffY = (lastpose.GetY() - currpose.GetY());
                double diffR = abs(lastpose.GetHeading() - currpose.GetHeading());

                cout << "lastpose : " << lastpose.GetX() << " " << lastpose.GetY() << " " << lastpose.GetHeading() << endl;
                cout << "currpose : " << currpose.GetX() << " " << currpose.GetY() << " " << currpose.GetHeading() << endl;
                cout << "poseDiff : " << diffX << " " << diffY << " " << diffR << endl << endl;

                covariance(0, 0) = 0.1 + diffX*diffX;
                covariance(1, 1) = 0.1 + diffY*diffY;
                covariance(0, 1) = covariance(1, 0) = diffX*diffY/10;
                covariance(2, 2) = 0.5 + diffR/2;

                pMapper->GetGraph()->LinkScans(pTempScan, pLocalizedRangeScan, currpose, covariance);
                pMapper->GetGraph()->LinkChainToScan(pMapper->GetMapperSensorManager()->GetRunningScans(name), pLocalizedRangeScan, currpose, covariance);
            }

            pMapper->GetMapperSensorManager()->AddRunningScan(pLocalizedRangeScan);

        }

        //std::cout << covariance(0, 0) << " " << covariance(0, 1) << " " << covariance(0, 2) << std::endl;
        //std::cout << covariance(1, 0) << " " << covariance(1, 1) << " " << covariance(1, 2) << std::endl;
        //std::cout << covariance(2, 0) << " " << covariance(2, 1) << " " << covariance(2, 2) << std::endl;

        if(pLocalizedRangeScan->GetStateId() == 884)
        {
            Matrix3 covariance2;
            covariance2.SetToIdentity();
            covariance2(0, 0) = 0.1;
            covariance2(1, 1) = 0.1;
            //covariance2(0, 1) = covariance2(1, 0) = 0.01;
            covariance2(2, 2) = 0.4;

            karto::LocalizedRangeScan* pTempScan = pMapper->GetMapperSensorManager()->GetScan(107);
            Pose2 posetemp = pTempScan->GetCorrectedPose();
            Pose2 posecur(posetemp.GetX(), posetemp.GetY(), posetemp.GetHeading() - 3.1415926/90);
            pLocalizedRangeScan->SetCorrectedPose(posecur);

            cout << "posetemp : " << posetemp.GetX() << " " << posetemp.GetY() << " " << posetemp.GetHeading() << endl;
            cout << "posecur : " << pLocalizedRangeScan->GetCorrectedPose().GetX() << " " << pLocalizedRangeScan->GetCorrectedPose().GetY() << " " << pLocalizedRangeScan->GetCorrectedPose().GetHeading() << endl;

            pMapper->GetGraph()->LinkScans(pTempScan, pLocalizedRangeScan, pLocalizedRangeScan->GetSensorPose(), covariance2);
            pMapper->GetGraph()->CorrectPoses();
        }

        if(pLocalizedRangeScan->GetStateId() == 0)
            pMapper->GetGraph()->CorrectPoses();

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        for(int i = 0; i < data.size(); i++)
        {
            if(i == 182 || i == 185)
                outfile << pLocalizedRangeScan->GetCorrectedPose().GetX() << " ";
            else if(i == 183 || i == 186)
                outfile << pLocalizedRangeScan->GetCorrectedPose().GetY() << " " ;
            else if(i == 184 || i == 187)
                outfile << pLocalizedRangeScan->GetCorrectedPose().GetHeading() << " ";
            else if(i == data.size()-1)
                outfile << data[i] << endl;
            else
                outfile << data[i] << " ";
        }

        int mapcount = 51;
        if(precess_state && mapcount > 50)
        {
            mapcount = 0;

            karto::OccupancyGrid* pOccupancyGrid = NULL;
            pOccupancyGrid = CreateOccupancyGrid(pMapper, occuGridResolution);

            kt_int32s width = pOccupancyGrid->GetWidth();
            kt_int32s height = pOccupancyGrid->GetHeight();
            //karto::Vector2<kt_double> offset = pOccupancyGrid->GetCoordinateConverter()->GetOffset();

            //std::cout << "width = " << width << ", height = " << height << ", scale = " << pOccupancyGrid->GetCoordinateConverter()->GetScale() << std::endl;

            Mat map(height, width, CV_8U);

            for (kt_int32s y = 0; y < height; y++)
            {
                for (kt_int32s x = 0; x < width; x++)
                {
                    // Getting the value at position x,y
                    kt_int8u value = pOccupancyGrid->GetValue(karto::Vector2<kt_int32s>(x, height - 1 - y));

                    switch (value)
                    {
                    case karto::GridStates_Unknown:
                        map.at<uchar>(y, x) = 255;
                        break;
                    case karto::GridStates_Occupied:
                        map.at<uchar>(y, x) = 255;
                        break;
                    case karto::GridStates_Free:
                        map.at<uchar>(y, x) = 255;
                        break;
                    case karto::GridStates_SensorPosition:
                        circle(map, Point(x, y), 1, Scalar(0), 1);
                        //map.data[y*width + x] = 0;
                        break;
                    default:
                        map.at<uchar>(y, x) = 255;
                        break;
                    }
                }
            }

            imshow("Image", map);

            char ch;
            if(step == false)
                ch = char(cvWaitKey(33));
            else
                ch = char(cvWaitKey(0));

            if (ch == 's')
                step = 1-step;
            else if (ch == 'q')
                return 0;
            else if (ch == 'p')
                cv::waitKey(0);

            delete pOccupancyGrid;

        }

		nCount++;
	}

    cout << "done..." << endl;

    karto::OccupancyGrid* pOccupancyGrid = NULL;
    pOccupancyGrid = CreateOccupancyGrid(pMapper, occuGridResolution);

    kt_int32s width = pOccupancyGrid->GetWidth();
    kt_int32s height = pOccupancyGrid->GetHeight();
    Mat map(height, width, CV_8UC3);

    for (kt_int32s y = 0; y < height; y++)
    {
        for (kt_int32s x = 0; x<width; x++)
        {
            // Getting the value at position x,y
            kt_int8u value = pOccupancyGrid->GetValue(karto::Vector2<kt_int32s>(x, height - 1 - y));

            switch (value)
            {
            case karto::GridStates_Unknown:
                map.at<Vec3b>(y, x)[0] = 255;
                map.at<Vec3b>(y, x)[1] = 255;
                map.at<Vec3b>(y, x)[2] = 255;
                break;
            case karto::GridStates_Occupied:
                map.at<Vec3b>(y, x)[0] = 255;
                map.at<Vec3b>(y, x)[1] = 255;
                map.at<Vec3b>(y, x)[2] = 255;
                break;
            case karto::GridStates_Free:
                map.at<Vec3b>(y, x)[0] = 255;
                map.at<Vec3b>(y, x)[1] = 255;
                map.at<Vec3b>(y, x)[2] = 255;
                break;
            case karto::GridStates_SensorPosition:
                circle(map, Point(x, y), 1, Scalar(0, 0, 255), 1);
                //map.data[y*width + x] = 0;
                break;
            default:
                map.at<Vec3b>(y, x)[0] = 255;
                map.at<Vec3b>(y, x)[1] = 255;
                map.at<Vec3b>(y, x)[2] = 255;
                break;
            }
        }
    }

    imshow("Image", map);

    imwrite("../log/" + fn2[0] + ".jpg", map);

    cout << "map is saved..." << endl;

    waitKey(0);

    delete pOccupancyGrid;
	delete pMapper;
	delete pDataset;
	return 0;
}




