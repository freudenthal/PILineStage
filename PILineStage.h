#ifndef PILineStage_h	//check for multiple inclusions
#define PILineStage_h

#include "Arduino.h"

#define PILineStageQueueCount 16
#define PILineReplyBufferCount 64

class PILineStage
{
	public:
		enum class CommandType : uint8_t
		{
			None,
			RequestStatus,
			RequestMotion,
			RequestReady,
			Stop,
			Acceleration,
			Deceleration,
			DefineHome,
			RequestError,
			FastReference,
			GoHome,
			Halt,
			Move,
			OnTarget,
			Position,
			StopAll,
			ServoMode,
			PositionMin,
			PositionMax,
			Velocity,
			Count
		};
		enum class CommandCategoryType : uint8_t
		{
			None,
			SingleCharacter,
			GetSet,
			Set,
			Get
		};
		enum class CommandParameterType : uint8_t
		{
			None,
			Index,
			Integer,
			Float,
			Bool,
			IndexInteger,
			IndexFloat,
			IndexBool
		};
		enum class CommandReplyType : uint8_t
		{
			None,
			StatusRegister,
			MotionStatus,
			ReadyStatus,
			ErrorStatus,
			IndexFloat,
			IndexBool
		};
		enum class ModeType : uint8_t
		{
			Inactive,
			Idle,
			SendMovePoll,
			RecieveMovePoll,
			WaitForReply,
			SendErrorPoll,
			ReceiveErrorPoll,
		};
		union CommandParameter
		{
			float FloatValue;
			int32_t IntegerValue;
			bool BoolValue;
		};
		struct CommandStruct
		{
			CommandType Command;
			const char* CommandString;
			CommandCategoryType Category;
			CommandParameterType SendType;
			CommandReplyType ReplyType;
			bool PollAfter;
		};
		struct CommandQueueEntry
		{
			CommandType Command;
			CommandStruct* CommandFormat;
			bool Get;
			uint8_t Axis;
			//bool PollStatus;
			CommandParameter Parameter;
		};
		typedef void ( *FinishedListener )();
		PILineStage(HardwareSerial* serial, int BaudRate); //Invoke with PILineStage(&SerialN);
		bool IsBusy();
		void Check();
		void Begin();
		void Home();
		bool GetIsHomed();
		bool GetIsHoming();
		bool SendHome(uint8_t Axis);
		bool SendMoveAbs(uint8_t Axis, float Position);
		bool SendGetPosition(uint8_t Axis);
		float GetPosition(uint8_t Axis);
		void SetFinishedCallback(FinishedListener Callback);
		void SetHomedCallback(FinishedListener Callback);
		void SetAxis1Callback(FinishedListener Callback);
		void SetAxis2Callback(FinishedListener Callback);
	private:
		void CheckCommandQueue();
		void Enqueue(CommandQueueEntry CommandToQueue);
		void ClearCommandQueue();
		bool SendCommand(CommandStruct* CommandToSend);
		bool CommandQueueFull();
		bool CommandQueueEmpty();
		uint8_t CommandQueueCount();
		void CommandQueueAdvance();
		void CommandQueueRetreat();
		void CommandQueuePut(CommandQueueEntry CommandPointer);
		bool CommandQueuePullToCurrentCommand();
		void CheckForCommandReply();
		void CheckForParameterReply();
		void ParseReplyData();
		void UpdateInternalVariables(CommandParameter* ParsedValue);
		void ModeTransitionToIdle();
		void CheckForStatus();
		void CheckForCompleted();
		void WaitToSendEcho();
		void SendCommandParameter();
		void SendMovePoll();
		void CheckForMoveComplete();
		void ModeTransitionToRecieveMovePoll();
		void ModeTransitionToSendMovePoll();
		void ModeTransitionToWaitForReply();
		void SendErrorPoll();
		void ModeTransitionToCheckErrors();
		void ModeTransitionToReceiveErrors();
		void CheckForErrorComplete();
		void SendCommand();
		void ParseReply();
		bool ParseStatusRegister(CommandParameter* ParameterToWrite);
		bool ParseMotionStatus(CommandParameter* ParameterToWrite);
		bool ParseReadyStatus(CommandParameter* ParameterToWrite);
		bool ParseErrorStatus(CommandParameter* ParameterToWrite);
		bool ParseIndexFloat(CommandParameter* ParameterToWrite);
		bool ParseIndexBool(CommandParameter* ParameterToWrite);
		int16_t IndexOf(char* CharArray, char ToFind, uint8_t Limit);
		void CenterStage();
		void FireCallbacks();
		static const CommandStruct CommandLibrary[];
		static const char EndOfLineCharacter;
		static const char SpaceCharacter;
		static const uint32_t ResetCompleteTime;
		static const uint32_t CommandReplyTimeMax;
		static const uint32_t TimeToCompleteDefault;
		static const uint8_t RetryCountMax;
		static const uint32_t WipeInputEvery;
		static const uint32_t PollTimeMax;
		HardwareSerial* SerialPort;
		FinishedListener FinishedCallback;
		FinishedListener HomedCallback;
		FinishedListener Axis1Callback;
		FinishedListener Axis2Callback;
		CommandQueueEntry CurrentCommand;
		uint32_t CurrentCommandTimeToComplete;
		uint32_t LastWipeTime;
		uint8_t ReplyByteCount;
		char ReplyData[PILineReplyBufferCount];
		CommandQueueEntry CommandQueue[PILineStageQueueCount];
		uint8_t CommandQueueHead;
		uint8_t CommandQueueTail;
		bool CommandQueueFullFlag;
		bool PollAfterQueueEmpty;
		uint8_t MotionByte;
		ModeType Mode;
		bool Busy;
		bool Ready;
		bool IsHomed;
		bool IsHoming;
		bool CheckForErrors;
		bool Axis1Moving;
		bool Axis2Moving;
		uint32_t PollTime;
		uint32_t ErrorCode;
		uint32_t LastCommandSentTime;
		uint32_t CommandReplyTime;
		uint32_t StatusByteReceivedTime;
		uint32_t ResetTime;
		uint8_t ResetWaitCount;
		uint32_t CommandRetryCount;
		uint32_t Status1;
		uint32_t Status2;
		float Position1;
		float Position2;
		float PositionMin1;
		float PositionMin2;
		float PositionMax1;
		float PositionMax2;
};
#endif
