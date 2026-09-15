using System.Collections.Generic;

namespace UnityRobotEnv.BDDL
{
    public abstract class SExpr
    {
        public override abstract string ToString();
    }

    public class SAtom : SExpr
    {
        public BDDLToken Token;

        public SAtom(BDDLToken token) { Token = token; }

        public string Value => Token.Value;
        public bool IsSymbol => Token.Type == BDDLTokenType.Symbol;
        public bool IsNumber => Token.Type == BDDLTokenType.Number;
        public bool IsString => Token.Type == BDDLTokenType.String;

        public override string ToString() => Token.Value;
    }

    public class SList : SExpr
    {
        public List<SExpr> Children;

        public SList() { Children = new List<SExpr>(); }
        public SList(List<SExpr> children) { Children = children; }

        public SExpr this[int i] => Children[i];
        public int Count => Children.Count;

        public override string ToString() =>
            "(" + string.Join(" ", Children.ConvertAll(c => c.ToString())) + ")";
    }
}
