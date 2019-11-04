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
			StopAll,
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
		}
		enum class CommandParameterType : uint8_t
		{
			None,
			Index,
			Integer,
			Float,
			Bool,
			IndexInteger,
			IndexFloat,
			IndexBool,
		};
		enum class CommandReplyType : uint8_t
		{
			None,
			StatusRegister,
			MotionStatus,
			ReadyStatus,
			ErrorStatus,
			IndexFloat,
			IndexBool,
		};
		enum class ModeType : uint8_t
		{
			Inactive,
			Idle,
			SendMovePoll,
			RecieveMovePoll,
			WaitForReply,
		};
		union CommandParameter
		{
			float FloatValue;
			int32_t IntegerValue;
			bool BoolValue
		}
		struct CommandStruct
		{
			CommandType Command;
			char* CommandString;
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
			bool PollStatus;
			CommandParameter Parameter;
		};
		typedef void ( *FinishedListener )();
		PILineStage(HardwareSerial* serial); //Invoke with PILineStage(&SerialN);
		bool IsBusy();
		void Check();
		void Begin();
		void Home();
		bool SendHome(uint8_t Axis);
		bool SendMoveAbs(uint8_t Axis, float Position);
		bool SendGetPosition(uint8_t Axis);
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
		void CommandQueuePut(CommandStruct* CommandPointer, uint32_t Parameter);
		bool CommandQueuePullToCurrentCommand();
		void CheckForCommandReply();
		void CheckForParameterReply();
		void ParseReplyData();
		void UpdateInternalVariables(uint32_t NewValue, CommandType PropertyToUpdate);
		void ModeTransitionToIdle();
		void CheckForStatus();
		void CheckForCompleted();
		void WaitToSendEcho();
		void SendCommandParameter();
		static const CommandStruct CommandLibrary[];
		static const char EndOfLineCharacter;
		static const char SpaceCharacter;
		static const uint32_t ResetCompleteTime;
		static const uint32_t CommandReplyTimeMax;
		static const uint32_t TimeToCompleteDefault;
		static const uint8_t RetryCountMax;
		static const uint32_t WipeInputEvery;
		HardwareSerial* SerialPort;
		FinishedListener RecievedCallback;
		CommandQueueEntry CurrentCommand;
		uint32_t CurrentCommandTimeToComplete;
		uint32_t LastWipeTime;
		uint8_t ReplyByteCount;
		uint8_t ReplyData[PILineReplyBufferCount];
		CommandQueueEntry CommandQueue[PILineQueueCount];
		uint8_t CommandQueueHead;
		uint8_t CommandQueueTail;
		bool CommandQueueFullFlag;
		bool PollAfterQueueEmpty;
		uint8_t MotionByte;
		ModeType Mode;
		bool Busy;
		bool Ready;
		bool IsHomed;
		uint32_t ErrorCode;
		uint32_t LastCommandSentTime;
		uint32_t CommandReplyTime;
		uint32_t StatusByteReceivedTime;
		uint32_t ResetTime;
		uint8_t ResetWaitCount;
		uint32_t CommandRetryCount;
		float PositionX;
		float PositionY;
};
#endif
