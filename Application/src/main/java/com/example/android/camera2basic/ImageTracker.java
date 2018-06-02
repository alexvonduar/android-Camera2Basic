package com.example.android.camera2basic;

import android.graphics.Bitmap;

public class ImageTracker {
    public static native boolean start();
    public static native boolean stop();
    public static native boolean track(byte[] img, int width, int height, int rotation, boolean is_nv21, boolean is_keyframe);
    //public static native boolean snap(byte[] img, int jpeg_size);
    public static native float[] getoverlaprect();
    public static native void renderPanorama(Bitmap bitmap);


    static {
        //System.loadLibrary("opencv_core");
        //System.loadLibrary("opencv_imgproc");
        //System.loadLibrary("opencv_features2d");
        //System.loadLibrary("opencv_calib3d");
        //System.loadLibrary("turbojpeg");
        System.loadLibrary("imagetracker");
    }

}

