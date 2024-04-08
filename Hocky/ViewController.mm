//
//  ViewController.m
//  Hocky
//
//  Created by milt on 12/14/23.
//

#define _DIVICE
#ifdef __cplusplus
#import <opencv2/opencv.hpp>
#endif
#ifdef __OBJC__
    #import <UIKit/UIKit.h>
    #import <Foundation/Foundation.h>
#endif
#ifdef _DIVICE
#import <opencv2/videoio/cap_ios.h>
#endif

#import "ViewController.hh"
using namespace cv;

@implementation ViewController{
//ß    IBOutlet UIButton* button;
#ifdef _DIVICE
    CvVideoCamera* videoCamera;
#endif
    int frame_number;
	double THRESOLD_VALUE;
	int thresold;
    int hit_frame;
    cv::Point hit_point;
    std::vector<cv::Rect> prev_boxes;
    bool videoFlag;
}

- (void)initTrack
{
    frame_number = 0;
    hit_frame = 0;
    hit_point = cv::Point(0, 0);
    prev_boxes.clear();
    videoFlag = false;
	THRESOLD_VALUE = 145.0;
	thresold = 160;
}

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view.
    [self initTrack];

#ifdef _DIVICE
    if (!videoFlag) {
        self.videoCamera = [[CvVideoCamera alloc] initWithParentView:self.cameraPreview];
        self.videoCamera.delegate = self;
        self.videoCamera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
        self.videoCamera.defaultAVCaptureSessionPreset = AVCaptureSessionPreset1920x1080;//AVCaptureSessionPreset352x288;
        self.videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationLandscapeLeft;
        self.videoCamera.defaultFPS = 30;
        self.videoCamera.grayscaleMode = NO;
    }
#endif
}

- (void)viewDidAppear:(BOOL)animated
{
    [super viewDidAppear:animated];
    
    dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
        if (videoFlag)
        {
            [self videoTest];
        } else {
            [self.videoCamera start];
        }
    });
}

- (void)viewDidDisappear:(BOOL)animated
{
    dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
        [self.videoCamera stop];
    });
    
    [super viewDidDisappear:animated];
}


#pragma mark - Protocol CvVideoCameraDelegate

- (void)processImage:(cv::Mat &)image
{
    //frame_number++;
    // tracking hocky
    [self trackingHocky:image];
    
    UIImage *uiImage = [self UIImageFromCVMat:image];
    dispatch_async(dispatch_get_main_queue(), ^{
        self.imageView.image = uiImage;
    });
}

#ifdef __cplusplus

#define THRESOLD_VALUE1 160
#define BALL_MAX_SIZE 30
#define BALL_MIN_AREA 36
#define BASE_WIDTH	600
#define BALL_GAP1	36
#define BALL_GAP2  36

int getThresold(const cv::Mat& image, cv::Rect rc1)
{
	int threshold = 0;
//	cv::Scalar pixelIntensity = cv::minMaxLoc(image(rc1), &minVal, &maxVal);
	cv::Scalar pixelIntensity = cv::mean(image(rc1));
	double val = 0.0;
	for (int i = 0; i < 4; i++) {
		val += pixelIntensity[i];
	}
	return (int)val;
}

bool isDarker(const cv::Mat & image, cv::Rect rc1, int thresold)
{
	int threshold = 30;
	// Get the intensity of the given pixel
	cv::Rect rc2;
	rc2.x = rc1.x + rc1.width / 4;
	rc2.y = rc1.y + rc1.height / 4;
	rc2.width = rc1.width - rc1.width / 2;
	rc2.height = rc1.height - rc1.height / 2;
	cv::Scalar pixelIntensity = cv::mean(image(rc1));
	double val1 = 0.0;
	double val2 = 0.0;
	for (int i = 0; i < 4/*pixelIntensity.channels()*/; i++) {
		val1 += pixelIntensity[i];
	}

	// Calculate the average intensity of the surrounding area
//	cv::Rect surround(rc1);
//	surround.y = rc1.y + rc1.height;
// 	cv::Scalar surroundingIntensity = cv::mean(image(surround));
// 	for (int i = 0; i < 4/*pixelIntensity.channels()*/; i++) {
// 		val2 += surroundingIntensity[i];
// 	}

	// Compare the pixel intensity with the surrounding area intensity
	if (val1 < thresold - thresold*0.16)// 25*3.0)
	{
		return true;
	}
	return false;
}
int getDistance(cv::Rect b1, cv::Rect b2, int* dx = 0, int* dy = 0)
{
	int x1 = b1.x, y1 = b1.y, w1 = b1.width, h1 = b1.height;
	int cx1 = x1 + w1 / 2, cy1 = y1 + h1 / 2;
	int x2 = b2.x, y2 = b2.y, w2 = b2.width, h2 = b2.height;
	int cx2 = x2 + w2 / 2, cy2 = y2 + h2 / 2;
	int xx = std::abs(cx1 - cx2);
	int yy = std::abs(cy1 - cy2);
	if (dx) *dx = xx;
	if (dy) *dy = yy;
	return xx * xx + yy * yy;
}
bool isPair(cv::Point o, cv::Point p1, cv::Point p2, int gap=10)
{
	int dx1 = (o.x - p1.x) * (o.x - p1.x);
	int dy1 = (o.y - p1.y) * (o.y - p1.y);
	int dx2 = (o.x - p2.x) * (o.x - p2.x);
	int dy2 = (o.y - p2.y) * (o.y - p2.y);
	if (std::abs(sqrt(dx1+dy1)-sqrt(dx2+dy2)) < gap)
		return true;
	return false;
}


-(void) videoTest
{
    NSString *nsString = [[NSBundle mainBundle] pathForResource:@"1" ofType:@"mp4"];
    std::string stdString = [nsString UTF8String];
    cv::String cvString(stdString);
    
    cv::VideoCapture cap(cvString);
    if (!cap.isOpened()) {
        //std::cout << "Error opening video file" << std::endl;
        return;
    }

    //cv::Ptr<cv::BackgroundSubtractorMOG2> fgbg = cv::createBackgroundSubtractorMOG2(150, 250, true);
    [self initTrack];
    int num_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    //std::cout << "frame: " << frame_number << "/" << num_frames;

    while (cap.isOpened()) {
        cv::Mat frame;
        bool ret = cap.read(frame);
        if (!ret) {
            break;
        }
        frame_number++;
        //std::cout << "\rframe: " << frame_number << "/" << num_frames;
        [self trackingHocky:frame];
        // Convert the cv::Mat to UIImage
        UIImage *uiImage = [self UIImageFromCVMat:frame];
        dispatch_async(dispatch_get_main_queue(), ^{
            self.imageView.image = uiImage;
        });
        
    }
}
- (UIImage *)UIImageFromCVMat:(cv::Mat)cvMat {
    NSData *data = [NSData dataWithBytes:cvMat.data length:cvMat.step.p[0]*cvMat.rows];

    CGColorSpaceRef colorSpace;
    CGBitmapInfo bitmapInfo;

    if (cvMat.elemSize() == 1) {
        colorSpace = CGColorSpaceCreateDeviceGray();
        bitmapInfo = kCGImageAlphaNone | kCGBitmapByteOrderDefault;
    } else {
        colorSpace = CGColorSpaceCreateDeviceRGB();
        bitmapInfo = kCGBitmapByteOrder32Little | (
            cvMat.elemSize() == 3 ? kCGImageAlphaNone : kCGImageAlphaNoneSkipFirst
        );
    }

    CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);

    // Creating CGImage from cv::Mat
    CGImageRef imageRef = CGImageCreate(
        cvMat.cols,                 //width
        cvMat.rows,                 //height
        8,                          //bits per component
        8 * cvMat.elemSize(),       //bits per pixel
        cvMat.step[0],              //bytesPerRow
        colorSpace,                 //colorspace
        bitmapInfo,                 // bitmap info
        provider,                   //CGDataProviderRef
        NULL,                       //decode
        false,                      //should interpolate
        kCGRenderingIntentDefault   //intent
    );

    // Getting UIImage from CGImage
    UIImage *finalImage = [UIImage imageWithCGImage:imageRef];
    CGImageRelease(imageRef);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);

    return finalImage;
}

- (void)trackingHocky:(Mat&)frame;
{
    frame_number++;
    //if (frame_number%2)
    //    return;
    //cv::Mat frame;
    //int imageWidth = frame.cols/2;
    //cv::resize(frame, frame, cv::Size(imageWidth, frame.rows * imageWidth / frame.cols));

    // Do some OpenCV stuff with the image
    //Mat frame_new;
    //cvtColor(frame, frame_new, COLOR_BGR2GRAY);
    cv::Mat frame_new;
    cv::cvtColor(frame, frame_new, cv::COLOR_BGRA2BGR);
    
    float scale = frame_new.cols * 1.0f / BASE_WIDTH;
    int ballMaxSize = (int)(BALL_MAX_SIZE * scale);
    int ballMinArea = (int)(BALL_MIN_AREA * scale * scale);
    int ballGap1 = (int)(BALL_GAP1 * scale);
    int ballGap2 = (int)(BALL_GAP2 * scale);
    int H = frame_new.rows, W = frame_new.cols;
	cv::Point leftTop(W * 0.41, H * 0.2);//5391,5392,5393
	cv::Point rightBottom(W * 0.78, H * 0.8);
        
    cv::Vec3b white = cv::Vec3b(255,255,255);
    setColColor(frame_new, 0, leftTop.x, white);
    setColColor(frame_new, rightBottom.x, W-1, white);
    setRowColor(frame_new, 0, leftTop.y, white);
    setRowColor(frame_new, rightBottom.y, H-1, white);
        
    std::vector<cv::Rect> cur_boxes;

    cv::GaussianBlur(frame_new, frame_new, cv::Size(5, 5), 0);
    cv::Mat gray = [self rgb_filter:frame_new];
    cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, gray, 0, 255, cv::THRESH_BINARY);// +cv::THRESH_OTSU);

    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(gray, labels, stats, centroids);

	if (frame_number == 3 || frame_number%300==0) {
		thresold = getThresold(frame_new, cv::Rect(leftTop, rightBottom));
		THRESOLD_VALUE = thresold*0.88/3;
	}
    std::vector<cv::Rect> boxes;
    for (int i = 0; i < num_labels; i++) {
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);

        if (w * h < ballMinArea) {
            continue;
        }

		if (h > ballMaxSize || w > ballMaxSize) {
			continue;
		}
		if (h / (float)w > 3.0f || w/(float)h > 3.0f) {
			continue;
		}
		cv::Rect rc(x, y, w, h);
		if (isDarker(frame_new, rc, thresold))
			boxes.push_back(rc);
	}
	//-----

	int gap = (int)(3*scale);
	if (prev_boxes.size() > 0)
		for (const auto& box : boxes) {
			int x = box.x, y = box.y, w = box.width, h = box.height;
			int cx = x + w / 2, cy = y + h / 2;
			bool dup = false;
			for (const auto& pbox : prev_boxes) {
				int px = pbox.x, py = pbox.y, pw = pbox.width, ph = pbox.height;
				int pcx = px + pw / 2, pcy = py + ph / 2;
				if ((std::abs(pcx - cx) < gap && std::abs(pcy - cy) < gap && std::abs(h-ph)<gap) && (w*h<150*scale*scale || ((float)w) / h <0.5f || ((float)w)/h>1.8f) ) {
					dup = true;
					break;
				}
			}
			if (!dup)
				cur_boxes.push_back(box);
		}
	else
		cur_boxes = boxes;
		for (const auto& box : cur_boxes) {
			cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);
		}

		bool is_hit = false;
		if (hit_frame == 0 || frame_number > hit_frame + 20) {
			int cur_boxes_size = (int)cur_boxes.size();
			int prev_boxes_size = (int)prev_boxes.size();
			/*if (cur_boxes_size==1 && prev_boxes_size > 1) {
				for (const auto& box : cur_boxes) {
					int x = box.x, y = box.y, w = box.width, h = box.height;
					int cx = x + w / 2, cy = y + h / 2;
					int cnt = 0;
					std::vector<cv::Point> candidates;
					for (const auto& prev_box : prev_boxes) {
						int px = prev_box.x, py = prev_box.y, pw = prev_box.width, ph = prev_box.height;
						int pcx = px + pw / 2, pcy = py + ph / 2;
						if (std::abs(cx - pcx) <= ballGap1 && std::abs(cy - pcy) <= ballGap1) {
							cnt++;
							candidates.push_back(cv::Point(pcx, pcy));
						}
					}
					if (cnt == 2 && isPair(cv::Point(cx, cy), candidates[0], candidates[1], 30 * scale)) {
						is_hit = true;
						int pcx = (candidates[0].x + candidates[1].x) / 2;
						int pcy = (candidates[0].y + candidates[1].y) / 2;
						int hx = pcx + (cx - pcx) * 1;
						int hy = pcy + (cy - pcy) * 1;
						hit_point = cv::Point(hx, hy);
					}
				}
			}*/
			if (!is_hit && cur_boxes_size > 1 && prev_boxes_size) {
				for (const auto& prev_box : prev_boxes) {
					int px = prev_box.x, py = prev_box.y, pw = prev_box.width, ph = prev_box.height;
					int pcx = px + pw / 2, pcy = py + ph / 2;
					int cnt = 0;
					std::vector<cv::Point> candidates;
					for (const auto& box : cur_boxes) {
						int x = box.x, y = box.y, w = box.width, h = box.height;
						int cx = x + w / 2, cy = y + h / 2;
						if (std::abs(cx - pcx) <= ballGap2 && std::abs(cy - pcy) <= ballGap2) {
							cnt++;
							candidates.push_back(cv::Point(cx, cy));
						}
					}
					if (cnt == 2 && isPair(cv::Point(pcx,pcy), candidates[0], candidates[1], 30*scale)) {
						is_hit = true;
						int cx = (candidates[0].x + candidates[1].x) / 2;
						int cy = (candidates[0].y + candidates[1].y) / 2;
						int hx = cx + (pcx - cx) * 2;
						int hy = cy + (pcy - cy) * 2;
						hit_point = cv::Point(hx, hy);
					}
				}
			}
			if (!is_hit && cur_boxes_size && prev_boxes_size==cur_boxes_size) {
				for (const auto& prev_box : prev_boxes) {
					int px = prev_box.x, py = prev_box.y, pw = prev_box.width, ph = prev_box.height;
					int pcx = px + pw / 2, pcy = py + ph / 2;
					int cnt = 0;
					cv::Rect curbox;
					for (const auto& box : cur_boxes) {
						int x = box.x, y = box.y, w = box.width, h = box.height;
						int cx = x + w / 2, cy = y + h / 2;
						if (std::abs(cx - pcx) < ballGap2 && std::abs(cy - pcy) < ballGap2) {
							cnt++;
							curbox = box;
						}
					}
					if (cnt == 1) { // larger
						if (curbox.area() > 280 * scale * scale && curbox.width>=18*scale && prev_box.area() * 1.1f < curbox.area() && prev_box.width * 1.1f < curbox.width)
						{
							is_hit = true;
							int cx = curbox.x + curbox.width / 2, cy = curbox.y + curbox.height / 2;
							int hx = (int)(cx + (pcx - cx) * 2.5f);
							int hy = (int)(cy + (pcy - cy) * 2.5f);
							hit_point = cv::Point(hx, hy);
						}
					}
				}
			}
		}
		if (is_hit) {
			hit_frame = frame_number;
		}

		if (hit_frame && frame_number < hit_frame + 15) {
			int x = hit_point.x;
			int y = hit_point.y;
			int rad = (int)(8 * scale);
			cv::ellipse(frame, cv::Point(x, y), cv::Size(rad, rad), 0, 0, 360, cv::Scalar(255, 0, 0), 3, LINE_4);
			//cv::rectangle(frame, cv::Point(x - 5 * scale, y - 5 * scale), cv::Point(x + 5 * scale, y + 5 * scale), cv::Scalar(0, 0, 255), 5);
		}

		prev_boxes = cur_boxes;
	//	prev_boxes0 = boxes;

    cv::rectangle(frame, leftTop, rightBottom, cv::Scalar(255, 0, 0), 2);
    cv::putText(frame, std::to_string(frame_number), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
    cv::putText(frame, std::to_string((int)THRESOLD_VALUE), cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    //cv::imshow("frame", frame);
    //cv::imshow("frame1", gray);
}

- (cv::Mat)rgb_filter:(Mat&)frame {
    cv::Mat mask;
    cv::inRange(frame, cv::Scalar(0, 0, 0), cv::Scalar(THRESOLD_VALUE-1, THRESOLD_VALUE-1, THRESOLD_VALUE-1), mask);
    cv::Mat result;
    cv::bitwise_and(frame, frame, result, mask);
    //cv::imshow("123", rgb_mask);
    return result;

//     cv::Mat mask = ~((rgb_mask.channels() < 3) |
//         ((rgb_mask.channels() == 3) &
//             (rgb_mask.ptr<cv::Vec3b>(0)[0] < r) &
//             (rgb_mask.ptr<cv::Vec3b>(0)[1] < g) &
//             (rgb_mask.ptr<cv::Vec3b>(0)[2] < b)));
//
//     cv::Mat white = cv::Mat::ones(frame.size(), frame.type()) * 255;
//     rgb_mask.setTo(white, mask);

}

void setRowColor(cv::Mat& frame, int startRow, int endRow, cv::Vec3b color)
{
    for (int row = startRow; row < endRow; row++) {
        for (int col = 0; col < frame.cols; col++) {
            frame.at<cv::Vec3b>(row, col) = color;
        }
    }
}
void setColColor(cv::Mat& frame, int startCol, int endCol, cv::Vec3b color)
{
    for (int col = startCol; col < endCol; col++) {
        for (int row = 0; row < frame.rows; row++) {
            frame.at<cv::Vec3b>(row, col) = color;
        }
    }
}

#endif

#pragma mark - UI Actions
- (IBAction)actionStart:(id)sender;
{
    [self initTrack];
    if (!videoFlag)
        [self videoTest];
#ifdef _DIVICE
        else
            [self.videoCamera start];
#endif
}

- (IBAction)onToggleCamera:(id)sender {
    AVCaptureDevicePosition position = self.videoCamera.defaultAVCaptureDevicePosition == AVCaptureDevicePositionFront ? AVCaptureDevicePositionBack : AVCaptureDevicePositionFront;
    
    dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), ^{
        [self.videoCamera stop];
        [self.videoCamera setDefaultAVCaptureDevicePosition:position];
        [self.videoCamera start];
    });
    
}

@end
