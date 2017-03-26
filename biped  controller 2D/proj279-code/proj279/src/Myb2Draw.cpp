#include "Myb2Draw.h"
#include "ofMain.h"



Myb2Draw::Myb2Draw() : of_app_(nullptr) {
	m_drawFlags = b2Draw::e_shapeBit | b2Draw::e_jointBit;
}

Myb2Draw::~Myb2Draw() {}

/// Draw a closed polygon provided in CCW order.
void Myb2Draw::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
}

/// Draw a solid closed polygon provided in CCW order.
void Myb2Draw::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
	ofSetColor(255 * color.r, 255 * color.g, 255 * color.b);
	ofVboMesh mesh;
	ofPath path;
	for (size_t i = 0; i < vertexCount; i++) {
		b2Vec2 tp = toScenePos(vertices[i]);
		path.lineTo(ofPoint(tp.x, tp.y));
	}
	mesh = path.getTessellation();
	mesh.setUsage(GL_STATIC_DRAW);

	ofPushMatrix();	
	//mesh.draw();
	mesh.drawWireframe();
	ofPopMatrix();
}

/// Draw a circle.
void Myb2Draw::DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color) {

}

/// Draw a solid circle.
void Myb2Draw::DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color) {
	b2Vec2 cp = toScenePos(center);
	radius *= of_app_->scale_;

	ofPushMatrix();
	ofNoFill();
	ofCircle(cp.x, cp.y, radius);
	ofPushStyle();
	ofSetColor(255 * color.r, 255 * color.g, 255 * color.b);
	ofEnableAlphaBlending();
	ofLine(0, 0, radius, 0);
	ofPopStyle();
	ofPopMatrix();
}

/// Draw a line segment.
void Myb2Draw::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) {
	ofSetColor(255 * color.r, 255 * color.g, 255 * color.b);
	ofSetLineWidth(2);
	b2Vec2 tp1 = toScenePos(p1);
	b2Vec2 tp2 = toScenePos(p2);
	ofLine(tp1.x, tp1.y, tp2.x, tp2.y);
}

/// Draw a transform. Choose your own length scale.
/// @param xf a transform.
void Myb2Draw::DrawTransform(const b2Transform& xf) {
}

const b2Vec2& Myb2Draw::toScenePos(const b2Vec2 &p)
{
	b2Vec2 tp;
	tp.x = p.x + of_app_->camera_pos_.x;
	tp.y = -1 * p.y + of_app_->camera_pos_.y;
	tp *= of_app_->scale_;
	return tp;
}
