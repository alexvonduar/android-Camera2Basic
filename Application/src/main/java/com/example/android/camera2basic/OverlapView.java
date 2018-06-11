package com.example.android.camera2basic;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.PointF;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;

public class OverlapView extends View {
    private int tl_x;
    private int tl_y;
    private int br_x;
    private int br_y;
    private int valid_overlap;
    private int valid_hint;
    private PointF[] points;
    private final Paint bgPaint;
    private final Paint fgPaint;
    //private FindHomography mFindHomography = null;
    //private ImageTracker mTracker;
    private int mWidth = 0;
    private int mHeight = 0;
    private Bitmap mPanoBitmap = null;
    public void setRectangle(int width, int height)
    {
        mWidth = height;
        mHeight = width;
    }

    //public void setmFindHomography(final FindHomography findHomography)
    //{
    //    mFindHomography = findHomography;
    //}
    //public void setTracker(final ImageTracker tracker)
    //{
    //    mTracker = tracker;
    //}

    public OverlapView(final Context context, final AttributeSet set) {
        super(context, set);

        fgPaint = new Paint();
        fgPaint.setAlpha(64);
        fgPaint.setColor(Color.RED);

        bgPaint = new Paint();
        bgPaint.setAlpha(64);
        bgPaint.setColor(Color.GREEN);
        this.tl_x = -1;
        this.tl_y = -1;
        this.br_x = -1;
        this.br_y = -1;
        //points = null;
    }

    public void setResults(final int tl_x, final int tl_y, final int br_x, final int br_y, final int valid) {
        this.tl_x = tl_x;
        this.tl_y = tl_y;
        this.br_x = br_x;
        this.br_y = br_y;
        this.valid_overlap = valid;
        postInvalidate();
    }

    public void setResults(final PointF[] points, final int valid) {
        //this.points = points;
        //this.valid = valid;
        postInvalidate();
    }

    public void draw() {
        //this.points = points;
        //this.valid = valid;
        postInvalidate();
    }

    private static final int PANO_HEIGHT = 250;

    @Override
    public void onDraw(final Canvas canvas) {
        Log.e("DEADBEAF", "onDraw++++");
        int canvasWidth = canvas.getWidth();
        int canvasHeight = canvas.getHeight();
        Paint panoPaint = new Paint();
        panoPaint.setAlpha(255);
        panoPaint.setColor(Color.DKGRAY);

        if (mPanoBitmap == null) {
            mPanoBitmap = Bitmap.createBitmap(canvasWidth, PANO_HEIGHT, Bitmap.Config.ARGB_8888);
            mPanoBitmap.eraseColor(Color.DKGRAY);
        }

        final int x = 10;
        float[] result = ImageTracker.getoverlaprect();
        int size = result.length;
        int num_points = (size - 2) / 2;
        if (num_points > 0) {
            points = new PointF[num_points];
            for (int i = 0; i < num_points; ++i) {
                points[i] = new PointF(result[i * 2] * mWidth, result[i * 2 + 1] * mHeight);
                Log.d("DEADBEAF", "get point" + points[i]);
            }
            valid_overlap = result[size - 2] > 0 ? 1 : 0;
            valid_hint = result[size - 1] > 0 ? 1 : 0;
        } else {
            points = null;
            valid_overlap = 0;
            valid_hint = 0;
        }

        //canvas.drawPaint(bgPaint);

        if (points != null) {
            Path path = new Path();
            path.moveTo(points[0].x, points[0].y);
            //int size = points.length;
            for (int i = 1; i < num_points; ++i) {
                path.lineTo(points[i].x, points[i].y);
            }
            path.lineTo(points[0].x, points[0].y);
            if (valid_overlap > 0) {
                canvas.drawPath(path, bgPaint);
            } else {
                canvas.drawPath(path, fgPaint);
            }
        } else {
            if (tl_x != -1) {

                //canvas.drawText(recog.getTitle() + ": " + recog.getConfidence(), x, y, fgPaint);
                if (valid_overlap > 0) {
                    canvas.drawRect(tl_x, tl_y, br_x, br_y, bgPaint);
                } else {
                    canvas.drawRect(tl_x, tl_y, br_x, br_y, fgPaint);
                }
            }
        }
        //canvas.drawRect(0, 0, canvasWidth, 250, panoPaint);
        ImageTracker.renderPanorama(mPanoBitmap);
        canvas.drawBitmap(mPanoBitmap, 0, 0, null);
        Log.e("DEADBEAF", "onDraw----");
    }
}
