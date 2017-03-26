#pragma once

#include "ofMain.h"
#include "Box2D.h"
#include "Myb2Draw.h"
#include "Myb2Listeners.h"
#include "Cartwheel/Character.h"
#include "Cartwheel/CharacterDescription.h"
#include "Cartwheel/Controller.h"
#include "Cartwheel/EditibleWalk.h"
#include "Cartwheel/World.h"


class Myb2Draw;

class ofApp : public ofBaseApp{

	public:
		ofApp();
		~ofApp();
		void setup() override;
		void update() override;
		void draw() override;

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void createMouseJoint(int x, int y);
		void updateMouse();

		b2Vec2 toBox2D(int x, int y);


		void initScene();
		void initCharacter();
		//void createMouseJoint();

		double last_step_time_;
		b2Vec2 last_mouse_pos_;
		b2Vec2 mouse_pos_;
		b2World world_;
		b2Body *floor_;
		b2Body *floor2_;
		
		// camera
		int camera_width_;
		int camera_height_;
		float32 scale_;
		b2Vec2 camera_pos_;
		Myb2Draw *my_draw_;
		MyDestructionListener* destruction_listerner_;
		MyContactFilter *contact_filter_;
		MyContactListener *contact_listener_;


		// Cartwheel
		CartwheelWorld cartwheel_world_;
		Character *character_;
		EditableWalking *controller_;
		
		// Control
		double push_time_;
		double push_force_;

		// Mouse joint
		b2MouseJoint *mouse_joint_;
		b2Body *mouse_body_;

		double hip_andle_, knee_angle_;

		friend int main();
		friend class Myb2Draw;


		
};


