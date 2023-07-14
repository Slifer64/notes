
from abc import abstractmethod, ABCMeta

#
#   Abstract Camera Class
#
class BaseCamera:

    # init internal RGB and D frames
    cv_rgb_image = None
    cv_depth_image = None

    def __init__(self, camera_ID, color_width, color_height, depth_width=None, depth_height = None, enable_depth=True):
        self.camera_ID = camera_ID
        self.enable_depth = enable_depth
        self.color_width = color_width
        self.color_height = color_height
        self.depth_width = depth_width
        self.depth_height = depth_height

        # colour and height are later aligned. Thus, both heights are identical
        self.width = color_width
        self.height = color_height

        if self.enable_depth:
            assert depth_width != None, "Depth is Enabled, yet no depth width is passed in BaseCamera Constructor"
            assert depth_height != None, "Depth is Enabled, yet no depth height is passed in BaseCamera Constructor"


    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def run(self):
        pass

    @abstractmethod
    def get_intrisincs(self):
        pass



    def get_color_frame(self):
        return self.cv_rgb_image

    def get_depth_frame(self):
        if self.enable_depth:
            return self.cv_depth_image
        else:
            return None


    def point_is_in_frame(self,x,y):
        A = x >= 0 and x < self.width
        B = y >= 0 and y < self.height
        return A and B


    def get_reverse_projection(self, x, y):
        # returns X,Y,Z
        pass

