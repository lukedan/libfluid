#pragma once

/// \file
/// Definition of the add_obstacle command.

#include <maya/MPxCommand.h>
#include <maya/MDGModifier.h>

namespace fluid::maya {
	/// The command used for adding an obstacle.
	class add_obstacle_command : public MPxCommand {
	public:
		static const MString name; ///< The name of this command.

		/// Creates an instance of this command.
		static void *creator();

		/// Creates the nodes.
		MStatus doIt(const MArgList&) override;
		/// Undoes the operation.
		MStatus undoIt() override;
		/// Redoes the operation.
		MStatus redoIt() override;

		/// This command can be undone.
		bool isUndoable() const override {
			return true;
		}
	private:
		MDGModifier _graph_modifier; ///< Used to create nodes and links.
	};
}
