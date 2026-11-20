package com.lineforge.lfr;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.RectF;
import android.view.View;

/** Small line icons for the bottom tab bar, drawn in code so they can be tinted. */
final class TabIcon extends View {
    static final int DRIVE = 0, TUNE = 1, CALIBRATE = 2, LOG = 3, RULES = 4;

    private final int kind;
    private final Paint stroke = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint fill = new Paint(Paint.ANTI_ALIAS_FLAG);

    TabIcon(Context context, int kind, int color) {
        super(context);
        this.kind = kind;
        stroke.setStyle(Paint.Style.STROKE);
        stroke.setStrokeCap(Paint.Cap.ROUND);
        stroke.setStrokeJoin(Paint.Join.ROUND);
        fill.setStyle(Paint.Style.FILL);
        setColor(color);
    }

    void setColor(int color) {
        stroke.setColor(color);
        fill.setColor(color);
        invalidate();
    }

    @Override protected void onDraw(Canvas c) {
        float s = Math.min(getWidth(), getHeight());
        float ox = (getWidth() - s) / 2f, oy = (getHeight() - s) / 2f;
        stroke.setStrokeWidth(s * 0.085f);
        float cx = ox + s / 2f, cy = oy + s / 2f;
        switch (kind) {
            case DRIVE: { // speed gauge
                float r = s * 0.38f;
                c.drawArc(new RectF(cx - r, cy - r, cx + r, cy + r), 150, 240, false, stroke);
                c.drawLine(cx, cy, cx + r * 0.55f, cy - r * 0.55f, stroke);
                c.drawCircle(cx, cy, s * 0.07f, fill);
                break;
            }
            case TUNE: { // three sliders with knobs
                float[] ys = {0.27f, 0.5f, 0.73f};
                float[] knobs = {0.35f, 0.66f, 0.46f};
                for (int i = 0; i < 3; i++) {
                    float y = oy + s * ys[i];
                    c.drawLine(ox + s * 0.14f, y, ox + s * 0.86f, y, stroke);
                    c.drawCircle(ox + s * knobs[i], y, s * 0.09f, fill);
                }
                break;
            }
            case CALIBRATE: { // crosshair
                float r = s * 0.25f;
                c.drawCircle(cx, cy, r, stroke);
                c.drawLine(cx, oy + s * 0.08f, cx, cy - r * 0.45f, stroke);
                c.drawLine(cx, cy + r * 0.45f, cx, oy + s * 0.92f, stroke);
                c.drawLine(ox + s * 0.08f, cy, cx - r * 0.45f, cy, stroke);
                c.drawLine(cx + r * 0.45f, cy, ox + s * 0.92f, cy, stroke);
                break;
            }
            case RULES: { // two toggle switches
                float h = s * 0.26f, w = s * 0.76f, x0 = ox + s * 0.12f;
                float y1 = oy + s * 0.18f, y2 = oy + s * 0.56f;
                c.drawRoundRect(new RectF(x0, y1, x0 + w, y1 + h), h / 2f, h / 2f, stroke);
                c.drawCircle(x0 + w - h / 2f, y1 + h / 2f, h * 0.3f, fill);
                c.drawRoundRect(new RectF(x0, y2, x0 + w, y2 + h), h / 2f, h / 2f, stroke);
                c.drawCircle(x0 + h / 2f, y2 + h / 2f, h * 0.3f, fill);
                break;
            }
            default: { // list
                float[] ys = {0.27f, 0.5f, 0.73f};
                for (float f : ys) {
                    float y = oy + s * f;
                    c.drawCircle(ox + s * 0.17f, y, s * 0.06f, fill);
                    c.drawLine(ox + s * 0.36f, y, ox + s * 0.86f, y, stroke);
                }
            }
        }
    }
}
