using System.Collections.Generic;

namespace UnityRobotEnv.BDDL
{
    public class BDDLSExprParser
    {
        private readonly List<BDDLToken> _tokens;
        private int _pos;

        public BDDLSExprParser(List<BDDLToken> tokens)
        {
            _tokens = tokens;
            _pos = 0;
        }

        private BDDLToken Current => _pos < _tokens.Count ? _tokens[_pos] : _tokens[_tokens.Count - 1];
        private BDDLToken Peek => _pos + 1 < _tokens.Count ? _tokens[_pos + 1] : _tokens[_tokens.Count - 1];

        public List<SExpr> ParseAll()
        {
            var exprs = new List<SExpr>();
            while (Current.Type != BDDLTokenType.EOF)
                exprs.Add(ParseExpr());
            return exprs;
        }

        public SExpr ParseExpr()
        {
            if (Current.Type == BDDLTokenType.EOF)
                return null;

            if (Current.Type == BDDLTokenType.LParen)
                return ParseList();
            return ParseAtom();
        }

        private SAtom ParseAtom()
        {
            var token = Current;
            _pos++;
            return new SAtom(token);
        }

        private SList ParseList()
        {
            int startLine = Current.Line, startCol = Current.Column;
            _pos++; // skip '('
            var children = new List<SExpr>();
            while (Current.Type != BDDLTokenType.RParen && Current.Type != BDDLTokenType.EOF)
                children.Add(ParseExpr());
            if (Current.Type == BDDLTokenType.EOF)
                throw new BDDLParseException("Unclosed parenthesis", startLine, startCol);
            _pos++; // skip ')'
            return new SList(children);
        }
    }
}
