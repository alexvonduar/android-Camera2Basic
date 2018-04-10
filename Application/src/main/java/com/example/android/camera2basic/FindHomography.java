package com.example.android.camera2basic;

public class FindHomography {

    public static native boolean start(int width, int height);
    public static native boolean stop();
    public static native boolean gethomography(byte[] img, int width, int height, int stride);
    public static native boolean setreference(byte[] img, int width, int height, int stride, int jpeg_size);
    public static native float[] getoverlaprect();


    static {
        //System.loadLibrary("opencv_core");
        //System.loadLibrary("opencv_imgproc");
        //System.loadLibrary("opencv_features2d");
        //System.loadLibrary("opencv_calib3d");
        //System.loadLibrary("turbojpeg");
        System.loadLibrary("find-homography");
    }

}
