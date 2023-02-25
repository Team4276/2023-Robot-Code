﻿using FRC.NetworkTables.Interop;
using System;
using System.Collections.Generic;
using System.Text;

namespace FRC.NetworkTables
{
    public readonly struct LogMessage
    {
        public readonly NtLogger Logger;
        public readonly LogLevel Level;
        public readonly string Filename;
        public readonly int Line;
        public readonly string Message;
        internal NetworkTableInstance Instance { get; }

        internal unsafe LogMessage(NetworkTableInstance inst, in NtLogMessage log)
        {
            Instance = inst;
            Logger = log.logger;
            Level = (LogLevel)log.level;
            Filename = UTF8String.ReadUTF8String(log.filename);
            Line = (int)log.line;
            Message = UTF8String.ReadUTF8String(log.message);
        }
    }
}
