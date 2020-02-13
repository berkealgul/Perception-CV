#include "Camera.hpp"

void Camera::processFrame(Mat frame, Mat prevImg, Mat R, Mat T, std::vector<Point2f> &prevFeatures)
{
	Mat currImg;
	//alýnan resmi siyah-beyaz formata dönüþtür
	std::vector<Point2f> currFeatures;
	std::vector<uchar>status;

	Mat mask;

	tractFeatures(prevImg, prevFeatures, currImg, currFeatures, status);

	Mat E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, currFeatures, prevFeatures, R, T, focal, pp, mask);

	if (prevFeatures.size() < MIN_FEAS)
	{
		detectFeatures(prevImg, prevFeatures);
		tractFeatures(prevImg, prevFeatures, &currImg, currFeatures, status);
	}

	*prevImg = currImg.clone();
	prevFeatures = currFeatures;
}

void Camera::detectFeatures(Mat inputImg, std::vector<Point2f> &points)
{
	std::vector<KeyPoint> keyPoints;
	FAST(inputImg, keyPoints, FastConst, true);

	//ileriki aþama için "keypoint" türü "point2f" türüne dönüþtürülmeli,
	KeyPoint::convert(keyPoints, points, std::vector<int>());
}

void Camera::tractFeatures(Mat prevImg, std::vector<Point2f> &prevPts, Mat currImg, std::vector<Point2f> &currPts, std::vector<uchar> &status)
{
	std::vector<float> err;
	calcOpticalFlowPyrLK(prevImg, currImg, prevPts, currPts, status, err, Size(21, 21), 3, termCrit, 0, 0.001);

	//hatalý eþleþmelerden kurtulur lakin
	//bu kýsým bana gizemli geldi o yüzden ayrýntýlar ile ilgili bir þey diyemeyeceðim
	int indexCorrection = 0;
	for (int i = 0; i < status.size(); i++)
	{
		Point2f pt = currPts.at(i - indexCorrection);
		if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
			if ((pt.x < 0) || (pt.y < 0)) {
				status.at(i) = 0;
			}
			currPts.erase(currPts.begin() + (i - indexCorrection));
			prevPts.erase(prevPts.begin() + (i - indexCorrection));
			indexCorrection++;
		}
	}
}

void Camera::update(float speed)
{
    if(speed <= 0.1)
    {
        cout << "Low Speed\n";
        return;
    }

    Mat dR, dT;
    Mat frame = getFrame();

    if (first == true)
    {
        detectFeatures(&prevImg, prevFeatures);
	    processFrame(&img2, &prevImg, &dR, &dT, prevFeatures);
        first = false;
    }

    processFrame(&img2, &prevImg, &dR, &dT, prevFeatures);

    if (T.at<double>(2) > T.at<double>(1)) && (T.at<double>(2) > T.at<double>(0))
    {
        T = T + speed * (R * dT);
    	R = dR * R;
    }
    else
        cout << "Invalid movement\n";
}

Camera::Camera(Mat _mtx, Mat _dist)
{
    cap = VideoCapture(0);
    mtx = _mtx;
    dist = _dist;

    Mat frame;
    cap.read(frame);

    float w = frame.width;
    float h = frame.height;
    float fx = mtx.at<double>(0,0);
    float cx = mtx.at<double>(0,2);
    float cy = mtx.at<double>(1,2)

    pp = Point2d(cx, cy);
    focal = fx / w;
}

Mat getFrame()
{
    Mat rawImage, frame;
    cap.read(rawImage);
    cvtColor(rawImage, frame, CV.COLOR_BGR2GRAY);
    undistort(frame, frame, mtx, dist);
    return frame;
}
