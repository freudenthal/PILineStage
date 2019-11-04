#include "PILineStage.h"
#include <stdio.h>

const char PILineStage::EndOfLineCharacter = '\n';
const char PILineStage::SpaceCharacter = ' ';
const uint32_t PILineStage::ResetCompleteTime = 1000000;
const uint32_t PILineStage::CommandReplyTimeMax = 250000;
const uint32_t PILineStage::TimeToCompleteDefault = 100000;
const uint8_t PILineStage::RetryCountMax = 8;
const uint32_t PILineStage::WipeInputEvery = 100000;
const PILineStage::CommandStruct PILineStage::CommandLibrary[] =
{
	{CommandType::None,"",CommandCategoryType::None,CommandParameterType::None,CommandReplyType::None,false},
	{CommandType::RequestStatus,"\x04",CommandCategoryType::SingleCharacter,CommandParameterType::None,CommandReplyType::StatusRegister,false},
	{CommandType::RequestMotion,"\x05",CommandCategoryType::SingleCharacter,CommandParameterType::None,CommandReplyType::MotionStatus,false},
	{CommandType::RequestReady,"\x07",CommandCategoryType::SingleCharacter,CommandParameterType::None,CommandReplyType::ReadyStatus,false},
	{CommandType::StopAll,"\x18",CommandCategoryType::SingleCharacter,CommandParameterType::None,CommandReplyType::None,false},
	{CommandType::Acceleration,"ACC",CommandCategoryType::GetSet,CommandParameterType::IndexFloat,CommandReplyType::IndexFloat,false},
	{CommandType::Deceleration,"DEC",CommandCategoryType::GetSet,CommandParameterType::IndexFloat,CommandReplyType::IndexFloat,false},
	{CommandType::DefineHome,"DFH",CommandCategoryType::GetSet,CommandParameterType::IndexFloat,CommandReplyType::IndexFloat,false},
	{CommandType::RequestError,"ERR",CommandCategoryType::Get,CommandParameterType::None,CommandReplyType::ErrorStatus,false},
	{CommandType::FastReference,"FRF",CommandCategoryType::GetSet,CommandParameterType::Index,CommandReplyType::IndexBool,true},
	{CommandType::GoHome,"GOH",CommandCategoryType::Set,CommandParameterType::Index,CommandReplyType::None,true},
	{CommandType::Halt,"HLT",CommandCategoryType::Set,CommandParameterType::Index,CommandReplyType::None,true},
	{CommandType::Move,"MOV",CommandCategoryType::SetGet,CommandParameterType::IndexFloat,CommandReplyType::IndexFloat,true},
	{CommandType::OnTarget,"ONT",CommandCategoryType::Get,CommandParameterType::Index,CommandReplyType::IndexBool,false},
	{CommandType::Position,"POS",CommandCategoryType::GetSet,CommandParameterType::IndexFloat,CommandReplyType::IndexFloat,false},
	{CommandType::StopAll,"STP",CommandCategoryType::Set,CommandParameterType::None,CommandReplyType::None,false},
	{CommandType::ServoMode,"SVO",CommandCategoryType::GetSet,CommandParameterType::IndexBool,CommandReplyType::IndexBool,false},
	{CommandType::PositionMin,"TMN",CommandCategoryType::Get,CommandParameterType::Index,CommandReplyType::IndexFloat,false},
	{CommandType::PositionMax,"TMX",CommandCategoryType::Get,CommandParameterType::Index,CommandReplyType::IndexFloat,false},
	{CommandType::Velocity,"VEL",CommandCategoryType::GetSet,CommandParameterType::IndexFloat,CommandReplyType::IndexFloat,false},
	{CommandType::Count,"",CommandCategoryType::None,CommandParameterType::None,CommandReplyType::None,false},
};
PILineStage::PILineStage(HardwareSerial *serial, int BaudRate)
{
	SerialPort = serial;
	SerialPort->begin(BaudRate, SERIAL_8N1);
	RecievedCallback = NULL;
	CurrentCommand = NULL;
	CurrentCommandParameter = 0;
	CurrentCommandTimeToComplete = 0;
	ReplyByteCount = 0;
	for (uint8_t Index = 0; Index < PILineReplyBufferCount; ++Index)
	{
		ReplyData[Index] = 0;
	}
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
	PollAfterQueueEmpty = false;
}
void PILineStage::Begin()
{
	ClearCommandQueue();
	Mode = ModeType::Idle;
}
void PILineStage::Home()
{
	CommandQueueEntry CommandToQueue;
	CommandToQueue.Command = CommandType::ServoMode;
	CommandToQueue.CommandFormat = NULL;
	CommandToQueue.Get = false;
	CommandToQueue.Axis = 1;
	CommandToQueue.Parameter = true;
	CommandToQueue.PollAfter = false;
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
	CommandToQueue.PollAfter = true;
	Enqueue(CommandToQueue);
}
void PILineStage::SetRecievedCallback(FinishedListener Finished)
{
	RecievedCallback = Finished;
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
		case ModeType::SendMovePoll:
			CheckForMoveComplete();
		case ModeType::WaitForReply:
			CheckForCommandReply();
			break;
		default:
			break;
	}
}
void PILineStage::SendMovePoll()
{
	MotionByte = 255;
	SerialPort->write('\x04');
	SerialPort->write(EndOfLineCharacter);
	LastCommandSentTime = micros();
	ModeTransitionToRecieveMovePoll();
}
void PILineStage::CheckCommandQueue()
{
	bool NewCommandPulled = CommandQueuePullToCurrentCommand();
	if (NewCommandPulled)
	{
		if (CurrentCommand != NULL)
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
	PILineStage::CommandQueuePut(CommandToQueue);
}
void PILineStage::ClearCommandQueue()
{
	for (int Index = 0; Index < PILineStageQueueCount; ++Index)
	{
		CommandQueue[Index].Command = CommandType::None;
		CommandQueue[Index].CommandFormat = NULL;
		CommandQueue[Index].Get = false;
		CommandQueue[Index].Axis = 0;
		CommandQueue[Index].Parameter = 0;
		CommandQueue[Index].PollAfter = false;
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
	CommandQueue[CommandQueueHead].PollStatus = NewCommand.PollStatus;
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
		CurrentCommand.PollStatus = CommandQueue[CommandQueueHead].PollStatus;
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
void PILineStage::CheckForMoveComplete()
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
				if (ReplyByteCount >= 1)
				{
					Serial.print("<PISTAGE>(Reply buffer overflow.)\n");
					ReplyByteCount = 0;
				}
			}
			else
			{
				if (ReplyData[0] == '0')
				{
					PollAfterQueueEmpty = false;
					ModeTransitionToIdle();
				}
				else if (ReplyData[0] == '1')
				{
					ModeTransitionToSendMovePoll();
				}
				else
				{
					Serial.print("<PISTAGE>(Unexpected reply on movement polling.)\n");
					ModeTransitionToIdle();
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
void PILineStage::ParseReply()
{
	bool Status = false;
	CommandParameter ReturnParameter;
	swtich (CurrentCommand.CommandFormat->ReplyType)
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
		case default:
			break;
	}
	if (!Status)
	{
		Serial.print("<PISTAGE>(Reply parse failed.)\n");
	}
}
bool PILineStage::ParseStatusRegister(CommandParameter* ParameterToWrite)
{
	char* Buffer[4];
	bool Status = false;
	Status &= (ReplyData[0] == '0');
	Status &= (ReplyData[1] == 'x');
	Status &= ReplyByteCount >= 6;
	if (Status && (ReplyByteCount >= 6))
	{
		memcpy(Buffer, ReplyData + 2, 4);
		Status1 = (uint32t)(atoi(Buffer));
	}
	if (Status && (ReplyByteCount >= 10))
	{
		memcpy(Buffer, ReplyData + 6, 4);
		Status2 = (uint32t)(atoi(Buffer));
	}
	if (Status && (ReplyByteCount >= 14))
	{
		memcpy(Buffer, ReplyData + 10, 4);
		Status3 = (uint32t)(atoi(Buffer));
	}
	if (Status == false)
	{
		Serial.print("<PISTAGE>(Status reply parse failed.)\n");
	}
	ParameterToWrite.IntegerValue = atoi(Buffer);
}
bool PILineStage::ParseMotionStatus(CommandParameter* ParameterToWrite)
{
	char MotionChar = ReplyData[0];
	MotionByte = atoi(ReplyData);
	ParameterToWrite.BoolValue = MotionByte == 0;
}
bool PILineStage::ParseReadyStatus(CommandParameter* ParameterToWrite)
{
	Ready = ReplyData[0] == 241;
	ParameterToWrite.BoolValue = Ready;
}
bool PILineStage::ParseErrorStatus(CommandParameter* ParameterToWrite)
{
	ErrorCode = atoi(ReplyData);
	ParameterToWrite.IntegerValue = ErrorCode;
}
bool PILineStage::ParseIndexFloat(CommandParameter* ParameterToWrite)
{
	int16_t EqualSplit = IndexOf(ReplyData,'=',PILineReplyBufferCount);
	if (EqualSplit == -1)
	{
		Serial.print("<PISTAGE>(Unable to find equal in reply parse.)\n");
	}
	else
	{
		ParameterToWrite.FloatValue = atof(EqualSplit + 1);
	}
}
bool PILineStage::ParseIndexBool(CommandParameter* ParameterToWrite)
{
	int16_t EqualSplit = IndexOf(ReplyData,'=',PILineReplyBufferCount);
	if (EqualSplit == -1)
	{
		Serial.print("<PISTAGE>(Unable to find equal in reply parse.)\n");
	}
	else
	{
		uint8_t Value = atoi(EqualSplit + 1);
		if (Value == 1)
		{
			ParameterToWrite.BoolValue = true;
		}
		else
		{
			ParameterToWrite.BoolValue = false;
		}
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
}
void PILineStage::ModeTransitionToRecieveMovePoll()
{
	memset(ReplyData, '\0', PILineReplyBufferCount);
	ReplyByteCount = 0;
	Mode = ModeType::RecieveMovePoll;
}
void PILineStage::ModeTransitionToWaitForReply()
{
	memset(ReplyData, '\0', PILineReplyBufferCount);
	ReplyByteCount = 0;
	Mode = ModeType::WaitForReply;
}
void PILineStage::ModeTransitionToIdle()
{
	if (CommandQueueEmpty())
	{
		if (PollAfterQueueEmpty)
		{
			ModeTransitionToSendMovePoll();
		}
		else
		{
			Busy = false;
			if (RecievedCallback != NULL)
			{
				RecievedCallback();
			}
			Mode = ModeType::Idle;
		}
	}
	else
	{
		Mode = ModeType::Idle;
	}
}
bool PILineStage::IsBusy()
{
	return Busy;
}
bool PILineStage::SendCommand()
{
	bool Status = true;
	char* OutputBuffer[PILineReplyBufferCount];
	uint8_t OutputBufferIndex = 0;
	if ( (CurrentCommand.Command == CommandType::None) || (CurrentCommand.Command == CommandType::Count) )
	{
		Serial.print("<MONOERROR>(Invalid command in buffer.)\n");
		Status = false;
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
	if (CurrentCommand.Get)
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
	)
	if (HasIndex)
	{
		OutputBuffer[OutputBufferIndex] = SpaceCharacter;
		OutputBufferIndex++;
		switch (CurrentCommand.Axis):
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
	if (CurrentCommand.CommandFormat->SendType != CommandParameterType::None)
	{
		OutputBuffer[OutputBufferIndex] = SpaceCharacter;
		OutputBufferIndex++;
		switch (CurrentCommand.CommandFormat->SendType):
		{
			case CommandParameterType::Float:
			case CommandParameterType::IndexFloat:
				OutputBufferIndex += sprintf(OutputBuffer + OutputBufferIndex,"%4.6f",CurrentCommand.Parameter.FloatValue);
				break;
			case CommandParameterType::Index:
			case CommandParameterType::Integer:
			case CommandParameterType::IndexInteger:
				OutputBufferIndex += sprintf(OutputBuffer + OutputBufferIndex,"%i",CurrentCommand.Parameter.IntegerValue);
				break;
			case CommandParameterType::Bool:
			case CommandParameterType::IndexBool:
				if (CurrentCommand.CommandFormat.Parameter.BoolValue)
				{
					OutputBuffer[OutputBufferIndex] = '1';
				}
				else
				{
					OutputBuffer[OutputBufferIndex] = '0';
				}
				OutputBufferIndex++;
				break;
			default:
				break;
		}
	}
	OutputBuffer[OutputBufferIndex] = EndOfLineCharacter;
	OutputBufferIndex++;
	OutputBuffer[OutputBufferIndex] = '\0';
	SerialPort->write(OutputBuffer,OutputBufferIndex);
	LastCommandSentTime = micros();
	if ( (CurrentCommand.CommandFormat->PollAfter) && CurrentCommand.PollAfter )
	{
		PollAfterQueueEmpty = true;
	}
	if (CurrentCommand.CommandFormat->ReplyType != CommandReplyType::None)
	{
		ModeTransitionToWaitForReply();
	}
	else
	{
		ModeTransitionToIdle();
	}
	return Status;
}
