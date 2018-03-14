/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "SurfaceReconstruction/Scene/Tree/NodesIterator.h"
#include "SurfaceReconstruction/Scene/Tree/Scope.h"
#include "SurfaceReconstruction/Scene/Tree/Tree.h"

using namespace std;
using namespace SurfaceReconstruction;

NodesIterator::NodesIterator(const Tree &tree, const NodesChecker *checker) :
	mNodes(tree.getNodes()), mChecker(checker)
{
	const Scope start = tree.getRootScope();

	// initialize stack
	mChildIndices.push_back(0);
	mScopes.push_back(start);

	// starting at an ignored node means starting at the end
	const bool isLeaf = mNodes.isLeaf(start.getNodeIndex());
	if (mChecker && mChecker->isIgnored(start, isLeaf, 0))
		popStack();
}

NodesIterator::~NodesIterator()
{
	mScopes.clear();
	mChildIndices.clear();
}

void NodesIterator::goToNext()
{
	// search until there is a unvisited node to which this iterator can go next
	while (!isAtTheEnd())
	{
		// depth first traversal
		if (goDown())
			return;

		// try sibling since current node does not allow to go down to any child
		popStack();
		if (isAtTheEnd())
			return;

		++mChildIndices.back();
	}
}

bool NodesIterator::goDown()
{
	assert(!isAtTheEnd());

	// maximum depth reached?
	const uint32 newChildDepth = (uint32) (mChildIndices.size());
	if (mChecker)
		if (newChildDepth > mChecker->getMaxDepth())
			return false;

	// any possible child left?
	if (isAtLeaf())
		return false;

	// test each child node: is the candidate child not ignored / the next one to go over?
	for (; mChildIndices.back() < Nodes::CHILD_COUNT; ++mChildIndices.back())
	{
		const uint32 targetChild = mChildIndices.back();
		const Scope childScope = mNodes.getChildScope(mScopes.back(), targetChild);
		const bool isLeaf = mNodes.isLeaf(childScope.getNodeIndex());
		if (mChecker && mChecker->isIgnored(childScope, isLeaf, newChildDepth))
			continue;

		mScopes.push_back(childScope);
		mChildIndices.push_back(0);
		return true;
	}

	// no fitting child
	return false;
}

void NodesIterator::popStack()
{
	if (isAtTheEnd())
		return;

	// one level up
	mScopes.pop_back();
	mChildIndices.pop_back();
}