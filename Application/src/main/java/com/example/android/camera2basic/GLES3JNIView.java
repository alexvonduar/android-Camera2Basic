package com.example.android.camera2basic;

import android.content.Context;
import android.graphics.PixelFormat;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.util.Log;
import android.view.SurfaceHolder;

import java.util.jar.Attributes;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

class GLES3JNIView extends GLSurfaceView {
    private static final String TAG = "GLES3JNI";
    private static final boolean DEBUG = true;

    public GLES3JNIView(Context context) {
        super(context);
        init(context);
    }

    public GLES3JNIView(Context context, AttributeSet attrs)
    {
        super(context, attrs);
        init(context);
    }

    public void init(Context context) {
        //super(context);
        // Pick an EGLConfig with RGB8 color, 16-bit depth, no stencil,
        // supporting OpenGL ES 2.0 or later backwards-compatible versions.
        setEGLConfigChooser(8, 8, 8, 8, 16, 0);
        setEGLContextClientVersion(3);
        setZOrderOnTop(true);
        setRenderer(new Renderer());
        getHolder().setFormat(PixelFormat.TRANSLUCENT);
    }

    public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {
        //
        Log.i(TAG, "DEADBEAF surfaceChanged " + w + " " + h);
        super.surfaceChanged(holder, format, w, h);
    }

    private static class Renderer implements GLSurfaceView.Renderer {
        public void onDrawFrame(GL10 gl) {
            GLES3JNILib.step();
        }

        public void onSurfaceChanged(GL10 gl, int width, int height) {
            Log.i(TAG, "DEADBEAF onSurfaceChanged " + width + " " + height);
            GLES3JNILib.resize(width, height);
        }

        public void onSurfaceCreated(GL10 gl, EGLConfig config) {
            GLES3JNILib.init();
        }
    }
}
