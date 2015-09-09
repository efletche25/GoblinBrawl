#pragma once
#include <functional>
#include <queue>
#include <map>

enum FSM_STATE {
	INITIALIZATION_DO_NOT_USE = 0, // This is required in case the first state is idle we still want the transition to happen
	IDLE,
	FORWARD,
	TURN_RIGHT,
	TURN_LEFT,
	BACKWARD,
	JUMP,
	FALL,
	DUCK,
	DIE,
	ATTACK,
	ATTACK_RIGHT,
	ATTACK_LEFT,
	ATTACK_JUMP,
};

/* Usage:
Intended to be used a member of a game object.

fsm = new FSM<GameObjectType>( this );
FSM<GameObjectType>::StateData idleStateData;
idleStateData.Before = &GameObjectType::Idle_Before;
idleStateData.Update = &GameObjectType::Idle_Update;
idleStateData.After = &GameObjectType::Idle_After;
fsm->AddState( FSM_STATE::IDLE, idleStateData );

Before and After functions will run once during a transition when Update is called
State.Update function will always once for each call to FSM.Update
Calling ChangeState multiple times between Updates is harmless but will do nothing except for the last one
*/
template<class T>
class FSM {
public:
	/*  When ChangeState is called the After callback of the current state will be
		added to the queue followed by Begin and State of the new State. */
	struct StateData {
		void(T::*Before)(float);	// optional
		void(T::*Update)(float);	// required
		void(T::*After)(float);		// optional
		StateData() : Before( nullptr ), Update( nullptr ), After( nullptr ) {};
	};

	FSM( T* _ownerInstance ) : ownerInstance( _ownerInstance ) { currentState = FSM_STATE::INITIALIZATION_DO_NOT_USE; };
	~FSM() {};
	void FSM::AddState( FSM_STATE state, StateData stateData ) {
		stateMap[state] = stateData;
	}

	void FSM::ChangeState( FSM_STATE _newState ) {
		newState = _newState;
	}

	void FSM::Update( float dt ) {
		if( newState!=currentState ) {
			if( activeQueue.size()>0 ) {
				activeQueue.pop();
			}
			auto currentStateIt = stateMap.find( currentState );
			if( currentStateIt!=stateMap.end() ) {
				if( currentStateIt->second.After ) {
					activeQueue.push( currentStateIt->second.After );
				}
			}
			auto newStateIt = stateMap.find( newState );
			if( newStateIt!=stateMap.end() ) {
				if( newStateIt->second.Before ) {
					auto x = newStateIt->second;
					auto y = x.Before;
					activeQueue.push( y );
				}
				assert( newStateIt->second.Update );
				auto q = newStateIt->second;
				auto r = q.Update;
				activeQueue.push( r );
			}
			currentState = newState;
		}
		assert( activeQueue.size()>0 );
		(ownerInstance->*activeQueue.front())(dt);
		while( activeQueue.size()>1 ) {
			activeQueue.pop();
			(ownerInstance->*activeQueue.front())(dt);
		};
	}

	T*									ownerInstance;
	FSM_STATE							currentState;
	FSM_STATE							newState;
	std::map<FSM_STATE, StateData>		stateMap;
	std::queue<void(T::*)(float)>		activeQueue;
};

