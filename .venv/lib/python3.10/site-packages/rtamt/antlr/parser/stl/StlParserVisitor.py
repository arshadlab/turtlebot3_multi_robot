# Generated from StlParser.g4 by ANTLR 4.7.2
from antlr4 import *

# This class defines a complete generic visitor for a parse tree produced by StlParser.

class StlParserVisitor(ParseTreeVisitor):

    # Visit a parse tree produced by StlParser#interval.
    def visitInterval(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#intervalTimeLiteral.
    def visitIntervalTimeLiteral(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#constantTimeLiteral.
    def visitConstantTimeLiteral(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#unit.
    def visitUnit(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprSince.
    def visitExprSince(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprParen.
    def visitExprParen(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprIff.
    def visitExprIff(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExpreOnce.
    def visitExpreOnce(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprEv.
    def visitExprEv(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprImplies.
    def visitExprImplies(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprUntil.
    def visitExprUntil(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprNot.
    def visitExprNot(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprNext.
    def visitExprNext(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprAnd.
    def visitExprAnd(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprUnless.
    def visitExprUnless(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprPrevious.
    def visitExprPrevious(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprHist.
    def visitExprHist(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprFall.
    def visitExprFall(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprPredicate.
    def visitExprPredicate(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprXor.
    def visitExprXor(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprRise.
    def visitExprRise(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprOr.
    def visitExprOr(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprAlways.
    def visitExprAlways(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprReal.
    def visitExprReal(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#specification_file.
    def visitSpecification_file(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#specification.
    def visitSpecification(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#SpecificationId.
    def visitSpecificationId(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#modImport.
    def visitModImport(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#assertion.
    def visitAssertion(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#declVariable.
    def visitDeclVariable(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#declConstant.
    def visitDeclConstant(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#annotation.
    def visitAnnotation(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#rosTopic.
    def visitRosTopic(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#variableDeclaration.
    def visitVariableDeclaration(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#constantDeclaration.
    def visitConstantDeclaration(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#AsgnLiteral.
    def visitAsgnLiteral(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#AsgnExpr.
    def visitAsgnExpr(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#domainType.
    def visitDomainType(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ioType.
    def visitIoType(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprSubtraction.
    def visitExprSubtraction(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprPow.
    def visitExprPow(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprDivision.
    def visitExprDivision(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprMultiplication.
    def visitExprMultiplication(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprLiteral.
    def visitExprLiteral(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprExp.
    def visitExprExp(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprSqrt.
    def visitExprSqrt(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprId.
    def visitExprId(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprAbs.
    def visitExprAbs(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#ExprAddition.
    def visitExprAddition(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#Leq.
    def visitLeq(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#Geq.
    def visitGeq(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#Less.
    def visitLess(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#Greater.
    def visitGreater(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#Eq.
    def visitEq(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#Neq.
    def visitNeq(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#literal.
    def visitLiteral(self, ctx):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by StlParser#Id.
    def visitId(self, ctx):
        return self.visitChildren(ctx)


