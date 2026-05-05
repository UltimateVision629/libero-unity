namespace LIBERO.BDDL
{
    public enum BDDLTokenType
    {
        LParen,         // (
        RParen,         // )
        Symbol,         // keyword, identifier, hyphenated-name
        String,         // "quoted string"
        Number,         // 1.0, -0.5, 3
        EOF
    }

    public struct BDDLToken
    {
        public BDDLTokenType Type;
        public string Value;
        public int Line;
        public int Column;

        public BDDLToken(BDDLTokenType type, string value, int line, int column)
        {
            Type = type;
            Value = value;
            Line = line;
            Column = column;
        }

        public override string ToString() =>
            $"<{Type}:'{Value}' @{Line}:{Column}>";
    }
}
