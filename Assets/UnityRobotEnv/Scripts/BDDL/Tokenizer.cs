using System.Collections.Generic;
using System.Text;

namespace UnityRobotEnv.BDDL
{
    public class BDDLTokenizer
    {
        private readonly string _input;
        private int _pos;
        private int _line;
        private int _col;

        public BDDLTokenizer(string input)
        {
            _input = input ?? "";
            _pos = 0;
            _line = 1;
            _col = 1;
        }

        public List<BDDLToken> TokenizeAll()
        {
            var tokens = new List<BDDLToken>();
            BDDLToken token;
            while ((token = NextToken()).Type != BDDLTokenType.EOF)
                tokens.Add(token);
            tokens.Add(token);
            return tokens;
        }

        public BDDLToken NextToken()
        {
            SkipWhitespaceAndComments();

            if (_pos >= _input.Length)
                return new BDDLToken(BDDLTokenType.EOF, "", _line, _col);

            char c = _input[_pos];

            switch (c)
            {
                case '(': _pos++; _col++; return new BDDLToken(BDDLTokenType.LParen, "(", _line, _col - 1);
                case ')': _pos++; _col++; return new BDDLToken(BDDLTokenType.RParen, ")", _line, _col - 1);
                case '"': return ReadString();
                default:
                    if (c == '-' && _pos + 1 < _input.Length && char.IsDigit(_input[_pos + 1]))
                        return ReadNumber();
                    if (char.IsDigit(c) || c == '-' || c == '+')
                        return ReadNumberOrSymbol();
                    if (IsSymbolChar(c))
                        return ReadSymbol();
                    throw new BDDLParseException($"Unexpected character '{c}'", _line, _col);
            }
        }

        private void SkipWhitespaceAndComments()
        {
            while (_pos < _input.Length)
            {
                char c = _input[_pos];
                if (char.IsWhiteSpace(c))
                {
                    if (c == '\n') { _line++; _col = 1; }
                    else _col++;
                    _pos++;
                }
                else if (c == ';')
                {
                    while (_pos < _input.Length && _input[_pos] != '\n')
                        _pos++;
                }
                else break;
            }
        }

        private BDDLToken ReadString()
        {
            int startLine = _line, startCol = _col;
            _pos++; _col++;
            var sb = new StringBuilder();
            while (_pos < _input.Length && _input[_pos] != '"')
            {
                if (_input[_pos] == '\\' && _pos + 1 < _input.Length)
                {
                    _pos++; _col++;
                    sb.Append(_input[_pos]);
                }
                else sb.Append(_input[_pos]);
                if (_input[_pos] == '\n') { _line++; _col = 1; }
                else _col++;
                _pos++;
            }
            if (_pos >= _input.Length)
                throw new BDDLParseException("Unterminated string", startLine, startCol);
            _pos++; _col++;
            return new BDDLToken(BDDLTokenType.String, sb.ToString(), startLine, startCol);
        }

        private BDDLToken ReadNumber()
        {
            int startLine = _line, startCol = _col;
            var sb = new StringBuilder();
            while (_pos < _input.Length && (char.IsDigit(_input[_pos]) || _input[_pos] == '.' ||
                   _input[_pos] == '-' || _input[_pos] == '+' || _input[_pos] == 'e' ||
                   _input[_pos] == 'E'))
            {
                sb.Append(_input[_pos]);
                _pos++; _col++;
            }
            return new BDDLToken(BDDLTokenType.Number, sb.ToString(), startLine, startCol);
        }

        private BDDLToken ReadNumberOrSymbol()
        {
            int startLine = _line, startCol = _col;
            int peek = _pos;
            int tempCol = _col;

            while (peek < _input.Length && (char.IsDigit(_input[peek]) || _input[peek] == '.' ||
                   _input[peek] == '-' || _input[peek] == '+' || _input[peek] == 'e' ||
                   _input[peek] == 'E'))
            {
                peek++; tempCol++;
            }

            bool hasDigit = false;
            for (int i = _pos; i < peek; i++)
            {
                if (char.IsDigit(_input[i])) hasDigit = true;
            }

            if (hasDigit && peek > _pos + 1)
            {
                var sb = new StringBuilder();
                for (int i = _pos; i < peek; i++)
                    sb.Append(_input[i]);
                _pos = peek; _col = tempCol;
                return new BDDLToken(BDDLTokenType.Number, sb.ToString(), startLine, startCol);
            }
            return ReadSymbol();
        }

        private BDDLToken ReadSymbol()
        {
            int startLine = _line, startCol = _col;
            var sb = new StringBuilder();
            while (_pos < _input.Length && IsSymbolChar(_input[_pos]))
            {
                sb.Append(_input[_pos]);
                _pos++; _col++;
            }
            return new BDDLToken(BDDLTokenType.Symbol, sb.ToString(), startLine, startCol);
        }

        private static bool IsSymbolChar(char c) =>
            !char.IsWhiteSpace(c) && c != '(' && c != ')' && c != '"' && c != ';';
    }
}
