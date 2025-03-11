# Generated from LtlParser.g4 by ANTLR 4.7.2
from antlr4 import *

# This class defines a complete generic visitor for a parse tree produced by LtlParser.

class LtlParserVisitor(ParseTreeVisitor):

    # Visit a parse tree produced by LtlParser#specification_file.
    def visitSpecification_file(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#specification.
    def visitSpecification(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#SpecificationId.
    def visitSpecificationId(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#modImport.
    def visitModImport(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#assertion.
    def visitAssertion(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#declVariable.
    def visitDeclVariable(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#declConstant.
    def visitDeclConstant(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#annotation.
    def visitAnnotation(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#rosTopic.
    def visitRosTopic(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#variableDeclaration.
    def visitVariableDeclaration(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#constantDeclaration.
    def visitConstantDeclaration(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#AsgnLiteral.
    def visitAsgnLiteral(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#AsgnExpr.
    def visitAsgnExpr(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#domainType.
    def visitDomainType(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ioType.
    def visitIoType(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprSince.
    def visitExprSince(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprParen.
    def visitExprParen(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprIff.
    def visitExprIff(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExpreOnce.
    def visitExpreOnce(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprEv.
    def visitExprEv(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprImplies.
    def visitExprImplies(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprUntil.
    def visitExprUntil(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprNot.
    def visitExprNot(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprNext.
    def visitExprNext(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprAnd.
    def visitExprAnd(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprUnless.
    def visitExprUnless(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprPrevious.
    def visitExprPrevious(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprHist.
    def visitExprHist(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprFall.
    def visitExprFall(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprPredicate.
    def visitExprPredicate(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprXor.
    def visitExprXor(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprRise.
    def visitExprRise(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprOr.
    def visitExprOr(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprAlways.
    def visitExprAlways(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprReal.
    def visitExprReal(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprSubtraction.
    def visitExprSubtraction(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprPow.
    def visitExprPow(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprDivision.
    def visitExprDivision(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprMultiplication.
    def visitExprMultiplication(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprLiteral.
    def visitExprLiteral(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprExp.
    def visitExprExp(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprSqrt.
    def visitExprSqrt(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprId.
    def visitExprId(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprAbs.
    def visitExprAbs(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#ExprAddition.
    def visitExprAddition(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#Leq.
    def visitLeq(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#Geq.
    def visitGeq(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#Less.
    def visitLess(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#Greater.
    def visitGreater(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#Eq.
    def visitEq(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#Neq.
    def visitNeq(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#literal.
    def visitLiteral(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by LtlParser#Id.
    def visitId(self, ctx):
        return self.visitChildren(ctx)


