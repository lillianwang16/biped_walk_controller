#include "ofApp.h"

ofApp::ofApp()
	: last_step_time_(double(GetTickCount()) / 1000.0),
	last_mouse_pos_(b2Vec2(-1.0, -1.0)),
	mouse_pos_(b2Vec2(-1.0, -1.0)),
	world_(b2Vec2(0.0, -9.8)),
	floor_(nullptr), floor2_(nullptr),
	camera_width_(1500), camera_height_(900),
	scale_(300.0),
	camera_pos_(b2Vec2(camera_width_ / 2.0 / scale_, camera_height_ / 1.3 / scale_)),
	my_draw_(new Myb2Draw()),
	destruction_listerner_(new MyDestructionListener()),
	contact_filter_(new MyContactFilter()),
	contact_listener_(new MyContactListener(&cartwheel_world_)),
	cartwheel_world_(&world_),
	character_(nullptr),
	controller_(nullptr),
	push_time_(0.0),
	push_force_(0.0),
	mouse_joint_(nullptr),
	mouse_body_(nullptr),
	hip_andle_(0.0), knee_angle_(0.0) {}

ofApp::~ofApp() {
	delete contact_listener_;
	delete contact_filter_;
	delete destruction_listerner_;
	delete my_draw_;
	world_.SetDebugDraw(nullptr);

}

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetVerticalSync(true);
	ofBackgroundHex(0xfdefc2);
	ofSetLogLevel(OF_LOG_NOTICE);
	ofDisableAntiAliasing();
	
	world_.SetDestructionListener(destruction_listerner_);
	world_.SetContactFilter(contact_filter_);
	world_.SetContactListener(contact_listener_);
	world_.SetDebugDraw(my_draw_);
	my_draw_->of_app_ = this;
	world_.SetWarmStarting(true);
	world_.SetContinuousPhysics(true);
	world_.SetAllowSleeping(true);

	b2BodyDef bd;
	mouse_body_ = world_.CreateBody(&bd);

	initScene();
	initCharacter();
	last_step_time_ = GetTickCount() / 1000.0;
}

void ofApp::initScene() {

	// create floor
	b2BodyDef bd;
	b2FixtureDef fd;
	b2EdgeShape edge;
	edge.Set(b2Vec2(-6.0, 0.0), b2Vec2(0.0, 0));
	fd.shape = &edge;
	fd.friction = 1.0;
	floor_ = world_.CreateBody(&bd);
	floor_->CreateFixture(&fd);

	b2BodyDef bd2;
	b2FixtureDef fd2;
	b2EdgeShape edge2;
	edge2.Set(b2Vec2(0.0, 0.1), b2Vec2(6.0, 0.1));
	fd2.shape = &edge2;
	fd2.friction = 1.0;
	floor2_ = world_.CreateBody(&bd2);
	floor2_->CreateFixture(&fd2);
	
	// create bridge
	b2BodyDef bd3;
	b2FixtureDef fd3;
	b2PolygonShape polygon;
	bd3.type = b2_dynamicBody;
	bd3.position = b2Vec2(-0.5, 0.5);
	polygon.SetAsBox(1, 0.01);
	fd3.shape = &polygon;
	fd3.density = 1.0;
	fd3.friction = 1.8;
	fd3.restitution = 0.35;
	b2Body *bridge = world_.CreateBody(&bd3);
	bridge->CreateFixture(&fd3);
	b2MassData md;
	md.mass = 20;
	md.I = 0.2;
	md.center = b2Vec2(0, 0);
	bridge->SetMassData(&md);
	bridge->SetLinearVelocity(b2Vec2(-0.0, -0.0));

	// create ball
	b2BodyDef bd4;
	b2FixtureDef fd4;
	b2CircleShape circle;
	bd4.type = b2_dynamicBody;
	bd4.position = b2Vec2(2.0, 1);
	circle.m_radius = 0.4;
	fd4.shape = &circle;
	fd4.density = 1.0;
	fd4.friction = 1.8;
	fd4.restitution = 0.35;
	b2Body *ball = world_.CreateBody(&bd4);
	ball->CreateFixture(&fd4);
	b2MassData md2;
	md2.mass = 600;
	md2.I = 0.2;
	md2.center = b2Vec2(0, 0);
	bridge->SetMassData(&md2);
	bridge->SetLinearVelocity(b2Vec2(-0.0, -0.0));
}

void ofApp::initCharacter() {
	CharacterDescription *chad = new CharacterDescription();
	character_ = chad->createCharacter(&world_, b2Vec2(-2.0, 0));
	cartwheel_world_.addArticulatedFigure(character_);
	delete chad;

	controller_ = new EditableWalking(character_);
	cartwheel_world_.addController(controller_);

	WorldOracle *world_oracle = new WorldOracle();
	world_oracle->InitializeWorld(&world_);

	BehaviourController *behaviour = new BehaviourController(
		character_, controller_, world_oracle);
	behaviour->initializeDefaultParameters();
	behaviour->requestVelocities(1.0);
	behaviour->requestStepTime(0.4);

	controller_->setBehaviour(behaviour);
	behaviour->conTransitionPlan();
}



void ofApp::update(){
	double one_step = 0.0005;
	double step = 0.01;
	double now_time = GetTickCount() / 1000.0;
	if (now_time - last_step_time_ < step) return;
	int count = (now_time - last_step_time_) / one_step;
	last_step_time_ = now_time;

	for (int step_count = count; step_count >= 1; --step_count) {		
		cartwheel_world_.performPreTasks(one_step);
		cartwheel_world_.advanceInTime(one_step);
		world_.Step(one_step, 1, 1);
		cartwheel_world_.performPostTasks(one_step);
	}
	//world_.DrawDebugData();
}

//--------------------------------------------------------------
void ofApp::draw(){
	world_.DrawDebugData();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
	mouse_pos_ = toBox2D(x, y);
	updateMouse();
}

b2Vec2 ofApp::toBox2D(int x, int y) {
	b2Vec2 p;
	p.x = double(x) / scale_ - camera_pos_.x;
	p.y = camera_pos_.y - double(y) / scale_;
	return p;
}

void ofApp::createMouseJoint(int x, int y) {
	
	b2AABB aabb;
	b2Body *body;

	b2Vec2 p = toBox2D(x, y);
	b2Vec2 d(0.001, 0.001);
	aabb.lowerBound = p - d;
	aabb.upperBound = p + d;

	QueryCallback qc(p);
	world_.QueryAABB(&qc, aabb);
	if (qc.fixture_ != nullptr) {
		body = qc.fixture_->GetBody();
		b2MouseJointDef md;
		md.bodyA = mouse_body_;
		md.bodyB = body;
		md.target = p;
		md.maxForce = 1000.0 * body->GetMass();
		mouse_joint_ = (b2MouseJoint*)world_.CreateJoint(&md);
		body->SetAwake(true);
	}
}

void ofApp::updateMouse() {
	if (mouse_joint_ != nullptr)
		mouse_joint_->SetTarget(mouse_pos_);
}
//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	mouse_pos_ = toBox2D(x, y);
	if (mouse_joint_ == nullptr) {
		createMouseJoint(x, y);
		updateMouse();
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
	if (mouse_joint_ != nullptr) {
		world_.DestroyJoint(mouse_joint_);
		mouse_joint_ = nullptr;
	}
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
