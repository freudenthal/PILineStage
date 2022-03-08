#include "PILineStage.h"
#include <stdio.h>

const char PILineStage::EndOfLineCharacter = '\n';
const char PILineStage::SpaceCharacter = ' ';
const uint32_t PILineStage::ResetCompleteTime = 1000000;
const uint32_t PILineStage::CommandReplyTimeMax = 250000;
const uint32_t PILineStage::TimeToCompleteDefault = 100000;
const uint8_t PILineStage::RetryCountMax = 8;
const uint32_t PILineStage::WipeInputEvery = 100000;
const uint32_t PILineStage::PollTimeMax = 100000;
const PILineStage::CommandStruct PILineStage::CommandLibrary[] =
{
	{CommandType::None,"",CommandCategoryType::None,CommandParameterType::None,CommandReplyType::None,false},
	{CommandType::RequestStatus,"\x04",CommandCategoryType::SingleCharacter,CommandParameterType::None,CommandReplyType::StatusRegister,false},
	{CommandType::RequestMotion,"\x05",CommandCategoryType::SingleCharacter,CommandParameterType::None,CommandReplyType::MotionStatus,false},
	{CommandType::RequestReady,"\x07",CommandCategoryType::SingleCharacter,CommandParameterType::None,CommandReplyType::ReadyStatus,false},
	{CommandType::Stop,"\x18",CommandCategoryType::SingleCharacter,CommandParameterType::None,CommandReplyType::None,false},
	{CommandType::Acceleration,"ACC",CommandCategoryType::GetSet,CommandParameterType::IndexFloat,CommandReplyType::IndexFloat,false},
	{CommandType::Deceleration,"DEC",CommandCategoryType::GetSet,CommandParameterType::IndexFloat,CommandReplyType::IndexFloat,false},
	{CommandType::DefineHome,"DFH",CommandCategoryType::GetSet,CommandParameterType::Index,CommandReplyType::IndexFloat,false},
	{CommandType::RequestError,"ERR",CommandCategoryType::Get,CommandParameterType::None,CommandReplyType::ErrorStatus,false},
	{CommandType::FastReference,"FRF",CommandCategoryType::GetSet,CommandParameterType::Index,CommandReplyType::None,true},
	{CommandType::GoHome,"GOH",CommandCategoryType::Set,CommandParameterType::Index,CommandReplyType::None,true},
	{CommandType::Halt,"HLT",CommandCategoryType::Set,CommandParameterType::Index,CommandReplyType::None,true},
	{CommandType::Move,"MOV",CommandCategoryType::GetSet,CommandParameterType::IndexFloat,CommandReplyType::IndexFloat,true},
	{CommandType::OnTarget,"ONT",CommandCategoryType::Get,CommandParameterType::Index,CommandReplyType::IndexBool,false},
	{CommandType::Position,"POS",CommandCategoryType::GetSet,CommandParameterType::IndexFloat,CommandReplyType::IndexFloat,false},
	{CommandType::StopAll,"STP",CommandCategoryType::Set,CommandParameterType::None,CommandReplyType::None,false},
	{CommandType::ServoMode,"SVO",CommandCategoryType::GetSet,CommandParameterType::IndexBool,CommandReplyType::None,false},
	{CommandType::PositionMin,"TMN",CommandCategoryType::Get,CommandParameterType::Index,CommandReplyType::IndexFloat,false},
	{CommandType::PositionMax,"TMX",CommandCategoryType::Get,CommandParameterType::Index,CommandReplyType::IndexFloat,false},
	{CommandType::Velocity,"VEL",CommandCategoryType::GetSet,CommandParameterType::IndexFloat,CommandReplyType::IndexFloat,false},
	{CommandType::Count,"",CommandCategoryType::None,CommandParameterType::None,CommandReplyType::None,false},
};
PILineStage::PILineStage(HardwareSerial *serial, int BaudRate)
{
	SerialPort = serial;
	SerialPort->begin(BaudRate, SERIAL_8N1);
	FinishedCallback = NULL;
	HomedCallback = NULL;
	Axis1Callback = NULL;
	Axis2Callback = NULL;
	ReplyByteCount = 0;
	for (uint8_t Index = 0; Index < PILineReplyBufferCount; ++Index)
	{
		ReplyData[Index] = 0;
	}
	CurrentCommand.Command = CommandType::None;
	CurrentCommand.CommandFormat = NULL;
	CurrentCommand.CompleteCallback = NULL;
	//CurrentCommand.PollStatus = false;
	CurrentCommand.Get = false;
	CurrentCommand.Axis = 0;
	CurrentCommand.Parameter.IntegerValue = 0;
	ClearCommandQueue();
	Mode = ModeType::Inactive;
	Busy = false;
	LastCommandSentTime = 0;
	CommandReplyTime = 0;
	StatusByteReceivedTime = 0;
	ResetTime = 0;
	ResetWaitCount = 0;
	CommandRetryCount = 0;
	MotionByte = 255;
	IsHomed = false;
	IsHoming = false;
	Status1 = 0;
	Status2 = 0;
	Position1 = 0.0;
	Position2 = 0.0;
	CheckForErrors = true;
	PositionMin1 = 0.0;
	PositionMin2 = 0.0;
	PositionMax1 = 0.0;
	PositionMax2 = 0.0;
	//PollAfterQueueEmpty = false;
}
void PILineStage::Begin()
{
	ClearCommandQueue();
	Mode = ModeType::Idle;
}
void PILineStage::SetVerbose(bool VerboseToSet)
{
	Verbose = VerboseToSet;
}
void PILineStage::Home()
{
	Axis1Moving = true;
	Axis2Moving = true;
	IsHomed = false;
	IsHoming = true;
	CommandQueueEntry CommandToQueue;
	CommandToQueue.Command = CommandType::RequestError;
	CommandToQueue.CommandFormat = NULL;
	CommandToQueue.CompleteCallback = NULL;
	CommandToQueue.Get = true;
	//CommandToQueue.PollStatus = false;
	Enqueue(CommandToQueue);
	CommandToQueue.Command = CommandType::ServoMode;
	CommandToQueue.CommandFormat = NULL;
	CommandToQueue.Get = false;
	CommandToQueue.Axis = 1;
	CommandToQueue.Parameter.BoolValue = true;
	//CommandToQueue.PollStatus = false;
	Enqueue(CommandToQueue);
	CommandToQueue.Axis = 2;
	Enqueue(CommandToQueue);
	CommandToQueue.Command = CommandType::FastReference;
	CommandToQueue.Axis = 1;
	Enqueue(CommandToQueue);
	CommandToQueue.Axis = 2;
	Enqueue(CommandToQueue);
	CommandToQueue.Command = CommandType::GoHome;
	CommandToQueue.Axis = 1;
	Enqueue(CommandToQueue);
	CommandToQueue.Axis = 2;
	Enqueue(CommandToQueue);
	CommandToQueue.Command = CommandType::PositionMin;
	CommandToQueue.Get = true;
	CommandToQueue.Axis = 1;
	Enqueue(CommandToQueue);
	CommandToQueue.Axis = 2;
	Enqueue(CommandToQueue);
	CommandToQueue.Command = CommandType::PositionMax;
	CommandToQueue.Axis = 1;
	Enqueue(CommandToQueue);
	CommandToQueue.Axis = 2;
	Enqueue(CommandToQueue);
	if (Verbose)
	{
		Serial.print("[STVERB](Homing.)\n");
	}
}
void PILineStage::CenterStage()
{
	/*
	Serial.print("<PISTAGE>(");
	Serial.print(PositionMin1);
	Serial.print(",");
	Serial.print(PositionMax1);
	Serial.print(",");
	Serial.print(PositionMin2);
	Serial.print(",");
	Serial.print(PositionMax2);
	Serial.print(")\n");
	*/
	float Center1 = (PositionMax1 - PositionMin1)/2.0;
	float Center2 = (PositionMax2 - PositionMin2)/2.0;
	CommandQueueEntry CommandToQueue;
	CommandToQueue.Command = CommandType::Move;
	CommandToQueue.CommandFormat = NULL;
	CommandToQueue.CompleteCallback = NULL;
	CommandToQueue.Get = false;
	CommandToQueue.Axis = 1;
	CommandToQueue.Parameter.FloatValue = Center1;
	Enqueue(CommandToQueue);
	CommandToQueue.Axis = 2;
	CommandToQueue.Parameter.FloatValue = Center2;
	Enqueue(CommandToQueue);
	CommandToQueue.Command = CommandType::DefineHome;
	CommandToQueue.Axis = 1;
	Enqueue(CommandToQueue);
	CommandToQueue.Command = CommandType::DefineHome;
	CommandToQueue.Axis = 2;
	Enqueue(CommandToQueue);
	CommandToQueue.Command = CommandType::Position;
	CommandToQueue.Get = true;
	CommandToQueue.Axis = 1;
	Enqueue(CommandToQueue);
	CommandToQueue.Axis = 2;
	Enqueue(CommandToQueue);
}
void PILineStage::SendGetIsMoving(FinishedListener Callback)
{
	CommandQueueEntry CommandToQueue;
	CommandToQueue.Command = CommandType::RequestMotion;
	CommandToQueue.CommandFormat = NULL;
	CommandToQueue.Get = true;
	CommandToQueue.Axis = 0;
	CommandToQueue.Parameter.BoolValue = false;
	CommandToQueue.CompleteCallback = Callback;
	Enqueue(CommandToQueue);
}
void PILineStage::SendGetPosition(uint8_t Axis, FinishedListener Callback)
{
	CommandQueueEntry CommandToQueue;
	CommandToQueue.Command = CommandType::Position;
	CommandToQueue.CommandFormat = NULL;
	CommandToQueue.Get = true;
	CommandToQueue.Axis = Axis;
	CommandToQueue.Parameter.BoolValue = false;
	CommandToQueue.CompleteCallback = Callback;
	//CommandToQueue.PollStatus = false;
	Enqueue(CommandToQueue);
}
float PILineStage::GetPosition(uint8_t Axis)
{
	if (Axis == 1)
	{
		return Position1;
	}
	else if (Axis == 2)
	{
		return Position2;
	}
	else
	{
		return 0;
	}
}
void PILineStage::SendMoveAbs(uint8_t Axis, float Position)
{
	CommandQueueEntry CommandToQueue;
	CommandToQueue.Command = CommandType::Move;
	CommandToQueue.CommandFormat = NULL;
	CommandToQueue.CompleteCallback = NULL;
	CommandToQueue.Get = false;
	CommandToQueue.Axis = Axis;
	CommandToQueue.Parameter.FloatValue = Position;
	Enqueue(CommandToQueue);
	CommandToQueue.Command = CommandType::Position;
	CommandToQueue.Get = true;
	CommandToQueue.Axis = Axis;
	Enqueue(CommandToQueue);
	if (Axis == 1)
	{
		Axis1Moving = true;
	}
	if (Axis == 2)
	{
		Axis2Moving = true;
	}
}
void PILineStage::SendSetVelocity(uint8_t Axis, float Velocity, FinishedListener Callback)
{
	CommandQueueEntry CommandToQueue;
	CommandToQueue.Command = CommandType::Velocity;
	CommandToQueue.CommandFormat = NULL;
	CommandToQueue.Get = false;
	CommandToQueue.Axis = Axis;
	CommandToQueue.Parameter.FloatValue = Velocity;
	CommandToQueue.CompleteCallback = Callback;
	Enqueue(CommandToQueue);
}
void PILineStage::SendGetVelocity(uint8_t Axis, FinishedListener Callback)
{
	CommandQueueEntry CommandToQueue;
	CommandToQueue.Command = CommandType::Velocity;
	CommandToQueue.CommandFormat = NULL;
	CommandToQueue.Get = true;
	CommandToQueue.Axis = Axis;
	CommandToQueue.Parameter.BoolValue = false;
	CommandToQueue.CompleteCallback = Callback;
	//CommandToQueue.PollStatus = false;
	Enqueue(CommandToQueue);
}
float PILineStage::GetVelocity(uint8_t Axis)
{
	if (Axis == 1)
	{
		return Velocity1;
	}
	else if (Axis == 2)
	{
		return Velocity2;
	}
	else
	{
		return 0;
	}
}
bool PILineStage::IsMoving(uint8_t Axis)
{
	if (Axis == 1)
	{
		return Axis1Moving;
	}
	else if (Axis == 2)
	{
		return Axis2Moving;
	}
	return false;
}
void PILineStage::SetFinishedCallback(FinishedListener Callback)
{
	FinishedCallback = Callback;
}
void PILineStage::SetHomedCallback(FinishedListener Callback)
{
	HomedCallback = Callback;
}
void PILineStage::SetAxisCompleteCallback(uint8_t Axis, FinishedListener Callback)
{
	if (Axis == 1)
	{
		SetAxis1Callback(Callback);
	}
	else if (Axis == 2)
	{
		SetAxis2Callback(Callback);
	}
}
void PILineStage::SetAxis1Callback(FinishedListener Callback)
{
	Axis1Callback = Callback;
}
void PILineStage::SetAxis2Callback(FinishedListener Callback)
{
	Axis2Callback = Callback;
}
void PILineStage::Check()
{
	switch (Mode)
	{
		case ModeType::Idle:
			CheckCommandQueue();
			break;
		case ModeType::SendMovePoll:
			SendMovePoll();
			break;
		case ModeType::RecieveMovePoll:
			CheckForMoveComplete();
			break;
		case ModeType::WaitForReply:
			CheckForCommandReply();
			break;
		case ModeType::SendErrorPoll:
			SendErrorPoll();
			break;
		case ModeType::ReceiveErrorPoll:
			CheckForErrorComplete();
			break;
		default:
			Serial.print("<PILINEERROR>(Mode not recognized.)\n");
			ModeTransitionToIdle();
			break;
	}
}
void PILineStage::SendMovePoll()
{
	if (micros() - PollTime > PollTimeMax)
	{
		PollTime = micros();
		MotionByte = 255;
		SerialPort->write('\x05');
		SerialPort->write(EndOfLineCharacter);
		//Serial.print('\x05');
		//Serial.print(EndOfLineCharacter);
		LastCommandSentTime = micros();
		ModeTransitionToRecieveMovePoll();
	}
}
void PILineStage::SendErrorPoll()
{
	if (micros() - PollTime > PollTimeMax)
	{
		PollTime = micros();
		ErrorCode = 0xFFFFFFFF;
		SerialPort->print("ERR?");
		SerialPort->print(EndOfLineCharacter);
		LastCommandSentTime = micros();
		ModeTransitionToReceiveErrors();
	}
}
void PILineStage::CheckCommandQueue()
{
	bool NewCommandPulled = CommandQueuePullToCurrentCommand();
	if (NewCommandPulled)
	{
		if ( (CurrentCommand.Command != CommandType::None) || (CurrentCommand.Command != CommandType::Count) )
		{
			Busy = true;
			SendCommand();
		}
		else
		{
			Serial.print("<MONOERROR>(Command in queue is null.)\n");
		}
	}
	if ( (micros() - LastWipeTime) > WipeInputEvery )
	{
		LastWipeTime = micros();
		if (SerialPort->available())
		{
			//uint8_t ByteRead = SerialPort->read();
			//Serial.print("J:");
			//Serial.print(ByteRead);
			//Serial.print("\n");
			SerialPort->read();
		}
	}
}
void PILineStage::Enqueue(CommandQueueEntry CommandToQueue)
{
	CommandToQueue.CommandFormat = const_cast<CommandStruct*>(&CommandLibrary[static_cast<uint8_t>(CommandToQueue.Command)]);
	CommandToQueue.Axis = constrain(CommandToQueue.Axis,0,3);
	CommandQueuePut(CommandToQueue);
}
void PILineStage::ClearCommandQueue()
{
	for (int Index = 0; Index < PILineStageQueueCount; ++Index)
	{
		CommandQueue[Index].Command = CommandType::None;
		CommandQueue[Index].CommandFormat = NULL;
		CommandQueue[Index].Get = false;
		CommandQueue[Index].Axis = 0;
		CommandQueue[Index].Parameter.IntegerValue = 0;
		CommandQueue[Index].CompleteCallback = NULL;
		//CommandQueue[Index].PollStatus = false;
	}
	CommandQueueHead = 0;
	CommandQueueTail = 0;
	CommandQueueFullFlag = false;
}
bool PILineStage::CommandQueueFull()
{
	return CommandQueueFullFlag;
}
bool PILineStage::CommandQueueEmpty()
{
	return ( !CommandQueueFullFlag && (CommandQueueHead == CommandQueueTail) );
}
uint8_t PILineStage::CommandQueueCount()
{
	uint8_t Count = PILineStageQueueCount;
	if(!CommandQueueFullFlag)
	{
		if(CommandQueueHead >= CommandQueueTail)
		{
			Count = (CommandQueueHead - CommandQueueTail);
		}
		else
		{
			Count = (PILineStageQueueCount + CommandQueueHead - CommandQueueTail);
		}
	}
	return Count;
}
void PILineStage::CommandQueueAdvance()
{
	if(CommandQueueFullFlag)
	{
		CommandQueueTail = (CommandQueueTail + 1) % PILineStageQueueCount;
	}
	CommandQueueHead = (CommandQueueHead + 1) % PILineStageQueueCount;
	CommandQueueFullFlag = (CommandQueueHead == CommandQueueTail);
}
void PILineStage::CommandQueueRetreat()
{
	CommandQueueFullFlag = false;
	CommandQueueTail = (CommandQueueTail + 1) % PILineStageQueueCount;
}
void PILineStage::CommandQueuePut(CommandQueueEntry NewCommand)
{
	CommandQueue[CommandQueueHead].Command = NewCommand.Command;
	CommandQueue[CommandQueueHead].CommandFormat = NewCommand.CommandFormat;
	CommandQueue[CommandQueueHead].Get = NewCommand.Get;
	CommandQueue[CommandQueueHead].Axis = NewCommand.Axis;
	CommandQueue[CommandQueueHead].Parameter = NewCommand.Parameter;
	CommandQueue[CommandQueueHead].CompleteCallback = NewCommand.CompleteCallback;
	//CommandQueue[CommandQueueHead].PollStatus = NewCommand.PollStatus;
	CommandQueueAdvance();
}
bool PILineStage::CommandQueuePullToCurrentCommand()
{
	bool Status = false;
	if (!CommandQueueEmpty())
	{
		CurrentCommand.Command = CommandQueue[CommandQueueTail].Command;
		CurrentCommand.CommandFormat = CommandQueue[CommandQueueTail].CommandFormat;
		CurrentCommand.Get = CommandQueue[CommandQueueTail].Get;
		CurrentCommand.Axis = CommandQueue[CommandQueueTail].Axis;
		CurrentCommand.Parameter = CommandQueue[CommandQueueTail].Parameter;
		CurrentCommand.CompleteCallback = CommandQueue[CommandQueueTail].CompleteCallback;
		//CurrentCommand.PollStatus = CommandQueue[CommandQueueHead].PollStatus;
		CommandQueueRetreat();
		Status = true;
	}
	return Status;
}
void PILineStage::CheckForCommandReply()
{
	if (SerialPort->available())
	{
		while(SerialPort->available())
		{
			char NewCharacter = SerialPort->read();
			if (NewCharacter != EndOfLineCharacter)
			{
				ReplyData[ReplyByteCount] = NewCharacter;
				ReplyByteCount++;
				if (ReplyByteCount >= PILineReplyBufferCount)
				{
					Serial.print("<PISTAGE>(Reply buffer overflow.)\n");
					ReplyByteCount = 0;
				}
			}
			else
			{
				if (Verbose)
				{
					Serial.print("[STVERB](Command reply: ");
					Serial.print(ReplyData);
					Serial.print(")\n");
				}
				ParseReply();
				ModeTransitionToIdle();
			}
		}
	}
	else if ( (micros() - LastCommandSentTime) > CommandReplyTimeMax)
	{
		Serial.print("<PISTAGE>(No command reply.)\n");
		ModeTransitionToIdle();
	}
}
bool PILineStage::GetIsHomed()
{
	return IsHomed;
}
bool PILineStage::GetIsHoming()
{
	return IsHoming;
}
void PILineStage::CheckForMoveComplete()
{
	if ( (micros() - PollTime > PollTimeMax) && (SerialPort->available()) )
	{
		PollTime = micros();
		while(SerialPort->available())
		{
			char NewCharacter = SerialPort->read();
			if (NewCharacter != EndOfLineCharacter)
			{
				ReplyData[ReplyByteCount] = NewCharacter;
				ReplyByteCount++;
				if (ReplyByteCount > 2)
				{
					Serial.print("<PISTAGE>(Move polling buffer overflow.)\n");
					ReplyData[ReplyByteCount]='\0';
					//Serial.print(ReplyData);
					//Serial.print("\n");
					ReplyByteCount = 0;
				}
			}
			else
			{
				if (ReplyData[0] == '0')
				{
					//PollAfterQueueEmpty = false;
					//Serial.print("<PISTAGE>(Move complete.)");
					if (Verbose)
					{
						Serial.print("[STVERB](Move reply idle: ");
						Serial.print(ReplyData);
						Serial.print(")\n");
					}
					ModeTransitionToIdle();
				}
				else
				{
					//Serial.print("<PISTAGE>(Moving.)");
					if (Verbose)
					{
						Serial.print("[STVERB](Move reply busy: ");
						Serial.print(ReplyData);
						Serial.print(")\n");
					}
					ModeTransitionToSendMovePoll();
				}
			}
		}
	}
	else if ( (micros() - LastCommandSentTime) > CommandReplyTimeMax)
	{
		Serial.print("<PISTAGE>(No movement complete reply.)\n");
		ModeTransitionToIdle();
	}
}
void PILineStage::CheckForErrorComplete()
{
	if ( (micros() - PollTime > PollTimeMax) && (SerialPort->available()) )
	{
		PollTime = micros();
		while(SerialPort->available())
		{
			char NewCharacter = SerialPort->read();
			if (NewCharacter != EndOfLineCharacter)
			{
				ReplyData[ReplyByteCount] = NewCharacter;
				ReplyByteCount++;
				if (ReplyByteCount > PILineReplyBufferCount)
				{
					Serial.print("<PISTAGE>(Error polling buffer overflow.)\n");
					ReplyData[ReplyByteCount]='\0';
					//Serial.print(ReplyData);
					//Serial.print("\n");
					ReplyByteCount = 0;
				}
			}
			else
			{
				ErrorCode = atoi(ReplyData);
				if (ErrorCode != 0)
				{
					Serial.print("<PISTAGE>(Error: ");
					Serial.print(ErrorCode);
					Serial.print(")\n");
				}
				if (Verbose)
				{
					Serial.print("[STVERB](Error check: ");
					Serial.print(ReplyData);
					Serial.print(")\n");
				}
				ModeTransitionToIdle();
			}
		}
	}
	else if ( (micros() - LastCommandSentTime) > CommandReplyTimeMax)
	{
		Serial.print("<PISTAGE>(No error status reply.)\n");
		ModeTransitionToIdle();
	}
}
void PILineStage::ParseReply()
{
	bool Status = false;
	CommandParameter ReturnParameter;
	switch (CurrentCommand.CommandFormat->ReplyType)
	{
		case(CommandReplyType::StatusRegister):
			Status = ParseStatusRegister(&ReturnParameter);
			break;
		case(CommandReplyType::MotionStatus):
			Status = ParseMotionStatus(&ReturnParameter);
			break;
		case(CommandReplyType::ReadyStatus):
			Status = ParseReadyStatus(&ReturnParameter);
			break;
		case(CommandReplyType::ErrorStatus):
			Status = ParseErrorStatus(&ReturnParameter);
			break;
		case(CommandReplyType::IndexFloat):
			Status = ParseIndexFloat(&ReturnParameter);
			break;
		case(CommandReplyType::IndexBool):
			Status = ParseIndexBool(&ReturnParameter);
			break;
		default:
			break;
	}
	if (Status)
	{
		UpdateInternalVariables(&ReturnParameter);
		if (Verbose)
		{
			Serial.print("[STVERB](Updated internals.)\n");
		}
	}
	else
	{
		Serial.print("<PISTAGE>(Reply parse failed.)\n");
	}
	if (CurrentCommand.CompleteCallback != NULL)
	{
		CurrentCommand.CompleteCallback();
		if (Verbose)
		{
			Serial.print("[STVERB](Command callback fired.)\n");
		}
	}
}
void PILineStage::UpdateInternalVariables(CommandParameter* Parameter)
{
	switch(CurrentCommand.Command)
	{
		case (CommandType::Move):
		case (CommandType::Position):
			{
				if (CurrentCommand.Axis == 1)
				{
					Position1 = Parameter->FloatValue;
				}
				else if (CurrentCommand.Axis == 2)
				{
					Position2 = Parameter->FloatValue;
				}
				break;
			}
		case (CommandType::PositionMin):
			{
				if (CurrentCommand.Axis == 1)
				{
					PositionMin1 = Parameter->FloatValue;
				}
				else if (CurrentCommand.Axis == 2)
				{
					PositionMin2 = Parameter->FloatValue;
				}
				break;
			}
		case (CommandType::PositionMax):
			{
				if (CurrentCommand.Axis == 1)
				{
					PositionMax1 = Parameter->FloatValue;
				}
				else if (CurrentCommand.Axis == 2)
				{
					PositionMax2 = Parameter->FloatValue;
				}
				break;
			}
		case (CommandType::Velocity):
			{
				if (CurrentCommand.Axis == 1)
				{
					Velocity1 = Parameter->FloatValue;
				}
				else if (CurrentCommand.Axis == 2)
				{
					Velocity2 = Parameter->FloatValue;
				}
				break;
			}
		default:
			break;
	}
}
bool PILineStage::ParseStatusRegister(CommandParameter* ParameterToWrite)
{
	char Buffer[4];
	bool Status = false;
	Status &= (ReplyData[0] == '0');
	Status &= (ReplyData[1] == 'x');
	Status &= ReplyByteCount >= 6;
	if (Status && (ReplyByteCount >= 6))
	{
		memcpy(Buffer, ReplyData + 2, 4);
		Status1 = (uint32_t)(atoi(Buffer));
	}
	if (Status && (ReplyByteCount >= 10))
	{
		memcpy(Buffer, ReplyData + 6, 4);
		Status2 = (uint32_t)(atoi(Buffer));
	}
	if (Status == false)
	{
		Serial.print("<PISTAGE>(Status reply parse failed.)\n");
	}
	ParameterToWrite->IntegerValue = atoi(Buffer);
	if (Verbose)
	{
		Serial.print("[STVERB](Status register: ");
		Serial.print(Status1);
		Serial.print(",");
		Serial.print(Status2);
		Serial.print(",");
		Serial.print(ParameterToWrite->IntegerValue);
		Serial.print(")\n");
	}
	return Status;
}
bool PILineStage::ParseMotionStatus(CommandParameter* ParameterToWrite)
{
	//0 - No axes in motion
	//1 - Axis 1 is in motion
	//2 - Axis 2 is in motion
	//3 - Axis 1 and 2 are in motion
	MotionByte = atoi(ReplyData);
	switch (MotionByte)
	{
		case 0:
			Axis1Moving = false;
			Axis2Moving = false;
			break;
		case 1:
			Axis1Moving = true;
			Axis2Moving = false;
			break;
		case 2:
			Axis1Moving = false;
			Axis2Moving = true;
			break;
		case 3:
			Axis1Moving = true;
			Axis2Moving = true;
			break;
		default:
			break;
	}
	ParameterToWrite->BoolValue = MotionByte == 0;
	if (Verbose)
	{
		Serial.print("[STVERB](Motion status: ");
		Serial.print(Axis1Moving);
		Serial.print(",");
		Serial.print(Axis2Moving);
		Serial.print(",");
		Serial.print(ParameterToWrite->BoolValue);
		Serial.print(")\n");
	}
	return true;
}
bool PILineStage::ParseReadyStatus(CommandParameter* ParameterToWrite)
{
	Ready = ReplyData[0] == 241;
	ParameterToWrite->BoolValue = Ready;
	if (Verbose)
	{
		Serial.print("[STVERB](Ready status: ");
		Serial.print(Ready);
		Serial.print(",");
		Serial.print(ParameterToWrite->BoolValue);
		Serial.print(")\n");
	}
	return true;
}
bool PILineStage::ParseErrorStatus(CommandParameter* ParameterToWrite)
{
	ErrorCode = atoi(ReplyData);
	ParameterToWrite->IntegerValue = ErrorCode;
	if (Verbose)
	{
		Serial.print("[STVERB](Error status: ");
		Serial.print(ErrorCode);
		Serial.print(",");
		Serial.print(ParameterToWrite->IntegerValue);
		Serial.print(")\n");
	}
	return true;
}
bool PILineStage::ParseIndexFloat(CommandParameter* ParameterToWrite)
{
	int16_t EqualSplit = IndexOf(ReplyData,'=',PILineReplyBufferCount);
	if (EqualSplit == -1)
	{
		Serial.print("<PISTAGE>(Unable to find equal in reply parse.)\n");
		return false;
	}
	else
	{
		ParameterToWrite->FloatValue = atof(ReplyData + EqualSplit + 1);
		if (Verbose)
		{
			Serial.print("[STVERB](Index float: ");
			Serial.print(ParameterToWrite->FloatValue);
			Serial.print(")\n");
		}
		return true;
	}
}
bool PILineStage::ParseIndexBool(CommandParameter* ParameterToWrite)
{
	int16_t EqualSplit = IndexOf(ReplyData,'=',PILineReplyBufferCount);
	if (EqualSplit == -1)
	{
		Serial.print("<PISTAGE>(Unable to find equal in reply parse.)\n");
		return false;
	}
	else
	{
		uint8_t Value = atoi(ReplyData + EqualSplit + 1);
		if (Value == 1)
		{
			ParameterToWrite->BoolValue = true;
		}
		else
		{
			ParameterToWrite->BoolValue = false;
		}
		if (Verbose)
		{
			Serial.print("[STVERB](Index bool: ");
			Serial.print(ParameterToWrite->BoolValue);
			Serial.print(")\n");
		}
		return true;
	}
}
int16_t PILineStage::IndexOf(char* CharArray, char ToFind, uint8_t Limit)
{
	int16_t Index = 0;
	while(true)
	{
		if (CharArray[Index] == ToFind)
		{
			return Index;
		}
		else if ( (CharArray[Index] == '\0') || (CharArray[Index] == EndOfLineCharacter) || (Index > Limit) )
		{
			return -1;
		}
		else
		{
			Index++;
		}
	}
}
void PILineStage::ModeTransitionToSendMovePoll()
{
	memset(ReplyData, '\0', PILineReplyBufferCount);
	ReplyByteCount = 0;
	Mode = ModeType::SendMovePoll;
	if (Verbose)
	{
		Serial.print("[STVERB](Transition: Send movement poll.)\n");
	}
}
void PILineStage::ModeTransitionToRecieveMovePoll()
{
	PollTime = micros();
	memset(ReplyData, '\0', PILineReplyBufferCount);
	ReplyByteCount = 0;
	Mode = ModeType::RecieveMovePoll;
	if (Verbose)
	{
		Serial.print("[STVERB](Transition: Receive movement poll.)\n");
	}
}
void PILineStage::ModeTransitionToWaitForReply()
{
	memset(ReplyData, '\0', PILineReplyBufferCount);
	ReplyByteCount = 0;
	Mode = ModeType::WaitForReply;
	if (Verbose)
	{
		Serial.print("[STVERB](Transition: Wait reply.)\n");
	}
}
void PILineStage::ModeTransitionToCheckErrors()
{
	PollTime = micros();
	memset(ReplyData, '\0', PILineReplyBufferCount);
	ReplyByteCount = 0;
	Mode = ModeType::SendErrorPoll;
	if (Verbose)
	{
		Serial.print("[STVERB](Transition: Send error poll.)\n");
	}
}
void PILineStage::ModeTransitionToReceiveErrors()
{
	PollTime = micros();
	memset(ReplyData, '\0', PILineReplyBufferCount);
	ReplyByteCount = 0;
	Mode = ModeType::ReceiveErrorPoll;
	if (Verbose)
	{
		Serial.print("[STVERB](Transition: Receive error poll.)\n");
	}
}
void PILineStage::ModeTransitionToIdle()
{
	if (CommandQueueEmpty())
	{
		if (CheckForErrors)
		{
			CheckForErrors = false;
			ModeTransitionToCheckErrors();
		}
		else
		{
			if (IsHoming)
			{
				CheckForErrors = true;
				IsHoming = false;
				CenterStage();
				Mode = ModeType::Idle;
			}
			else
			{
				CheckForErrors = true;
				Busy = false;
				FireCallbacks();
				Mode = ModeType::Idle;
				if (Verbose)
				{
					Serial.print("[STVERB](Transition: Idle with callbacks.)\n");
				}
			}
		}
	}
	else
	{
		Mode = ModeType::Idle;
		if (Verbose)
		{
			Serial.print("[STVERB](Transition: Idle send next.)\n");
		}
	}
}
void PILineStage::FireCallbacks()
{
	if (!IsHomed)
	{
		IsHomed = true;
		if (HomedCallback != NULL)
		{
			HomedCallback();
		}
	}
	if (Axis1Moving)
	{
		Axis1Moving = false;
		if (Axis1Callback != NULL)
		{
			Axis1Callback();
		}
	}
	if (Axis2Moving)
	{
		Axis2Moving = false;
		if (Axis2Callback != NULL)
		{
			Axis2Callback();
		}
	}
	if (FinishedCallback != NULL)
	{
		FinishedCallback();
	}
}
bool PILineStage::IsBusy()
{
	return Busy;
}
void PILineStage::SendCommand()
{
	char OutputBuffer[PILineReplyBufferCount];
	uint8_t OutputBufferIndex = 0;
	if ( (CurrentCommand.Command == CommandType::None) || (CurrentCommand.Command == CommandType::Count) )
	{
		Serial.print("<MONOERROR>(Invalid command in buffer.)\n");
	}
	else if (CurrentCommand.CommandFormat->Category == CommandCategoryType::SingleCharacter)
	{
		OutputBuffer[OutputBufferIndex] = CurrentCommand.CommandFormat->CommandString[0];
		OutputBufferIndex++;
	}
	else
	{
		memcpy(OutputBuffer,CurrentCommand.CommandFormat->CommandString,3);
		OutputBufferIndex += 3;
	}
	if ( (CurrentCommand.CommandFormat->Category != CommandCategoryType::SingleCharacter) && CurrentCommand.Get)
	{
		OutputBuffer[OutputBufferIndex] = '?';
		OutputBufferIndex++;
	}
	bool HasIndex =
	(
		(CurrentCommand.CommandFormat->SendType == CommandParameterType::Index) ||
		(CurrentCommand.CommandFormat->SendType == CommandParameterType::IndexFloat) ||
		(CurrentCommand.CommandFormat->SendType == CommandParameterType::IndexInteger) ||
		(CurrentCommand.CommandFormat->SendType == CommandParameterType::IndexBool)
	);
	if (HasIndex)
	{
		OutputBuffer[OutputBufferIndex] = SpaceCharacter;
		OutputBufferIndex++;
		switch (CurrentCommand.Axis)
		{
			case 0:
				OutputBuffer[OutputBufferIndex] = '0';
				break;
			case 1:
				OutputBuffer[OutputBufferIndex] = '1';
				break;
			case 2:
				OutputBuffer[OutputBufferIndex] = '2';
				break;
			case 3:
				OutputBuffer[OutputBufferIndex] = '3';
				break;
			default:
				OutputBufferIndex--;
				break;
		}
		OutputBufferIndex++;
	}
	if ( (CurrentCommand.CommandFormat->SendType != CommandParameterType::None) && !CurrentCommand.Get)
	{
		OutputBuffer[OutputBufferIndex] = SpaceCharacter;
		OutputBufferIndex++;
		switch (CurrentCommand.CommandFormat->SendType)
		{
			case CommandParameterType::Float:
			case CommandParameterType::IndexFloat:
				OutputBufferIndex += sprintf(OutputBuffer + OutputBufferIndex,"%4.6f",(float)(CurrentCommand.Parameter.FloatValue));
				break;
			case CommandParameterType::Integer:
			case CommandParameterType::IndexInteger:
				OutputBufferIndex += sprintf(OutputBuffer + OutputBufferIndex,"%i",(int)(CurrentCommand.Parameter.IntegerValue));
				break;
			case CommandParameterType::Bool:
			case CommandParameterType::IndexBool:
				{
					if (CurrentCommand.Parameter.BoolValue)
					{
						OutputBuffer[OutputBufferIndex] = '1';
					}
					else
					{
						OutputBuffer[OutputBufferIndex] = '0';
					}
					OutputBufferIndex++;
					break;
				}
			default:
				break;
		}
	}
	OutputBuffer[OutputBufferIndex] = EndOfLineCharacter;
	OutputBufferIndex++;
	OutputBuffer[OutputBufferIndex] = '\0';
	SerialPort->write(OutputBuffer,OutputBufferIndex);
	if (Verbose)
	{
		Serial.print("[STVERB](Send: ");
		Serial.print(OutputBuffer);
		Serial.print(")\n");
	}
	//Serial.print("<PISTAGE>(");
	//Serial.print(OutputBuffer);
	//Serial.print(")\n");
	LastCommandSentTime = micros();
	if ( (CurrentCommand.CommandFormat->ReplyType != CommandReplyType::None) && CurrentCommand.Get )
	{
		ModeTransitionToWaitForReply();
	}
	else if (CurrentCommand.CommandFormat->PollAfter)
	{
		ModeTransitionToSendMovePoll();
	}
	else
	{
		ModeTransitionToIdle();
	}
}
