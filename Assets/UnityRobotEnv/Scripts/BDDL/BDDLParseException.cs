using System;

namespace UnityRobotEnv.BDDL
{
    public class BDDLParseException : Exception
    {
        public int Line { get; }
        public int Column { get; }

        public BDDLParseException(string message, int line, int col) : base($"[{line}:{col}] {message}")
        {
            Line = line;
            Column = col;
        }
    }
}
