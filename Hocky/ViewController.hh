//
//  ViewController.h
//  Hocky
//
//  Created by milt on 12/14/23.
//

#import <UIKit/UIKit.h>

@interface ViewController : UIViewController<CvVideoCameraDelegate>

#ifdef _DIVICE
@property (nonatomic, retain) CvVideoCamera* videoCamera;
#endif
@property (nonatomic, weak) IBOutlet UIImageView *imageView;
@property (nonatomic, weak) IBOutlet UIView *cameraPreview;

- (IBAction)actionStart:(id)sender;
- (IBAction)onToggleCamera:(id)sender;

@end

