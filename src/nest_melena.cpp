/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.
   
   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/lgpl.html>.
   
   Author:     Helge Mathee      helge.mathee@gmx.net
   Company:    Studio Nest (TM)
   Date:       2010 / 09 / 21
*/

#include <xsi_application.h>
#include <xsi_context.h>
#include <xsi_pluginregistrar.h>
#include <xsi_status.h>

#include <xsi_icenodecontext.h>
#include <xsi_icenodedef.h>
#include <xsi_command.h>
#include <xsi_factory.h>
#include <xsi_math.h>
#include <xsi_vector2f.h>
#include <xsi_vector3f.h>
#include <xsi_vector4f.h>
#include <xsi_matrix3f.h>
#include <xsi_matrix4f.h>
#include <xsi_rotationf.h>
#include <xsi_quaternionf.h>
#include <xsi_color4f.h>
#include <xsi_shape.h>
#include <xsi_indexset.h>
#include <xsi_floatarray.h>
#include <xsi_dataarray.h>
#include <xsi_dataarray2D.h>
#include <xsi_doublearray.h>
#include <xsi_x3dobject.h>
#include <xsi_primitive.h>
#include <xsi_selection.h>
#include <xsi_customoperator.h>
#include <xsi_ppglayout.h>
#include <xsi_menu.h>
#include <xsi_menuitem.h>
#include <xsi_polygonmesh.h>
#include <xsi_operatorcontext.h>
#include <xsi_iceattribute.h>
#include <xsi_iceattributedataarray.h>
#include <xsi_iceattributedataarray2D.h>
#include <xsi_nurbscurvelist.h>
#include <xsi_hairprimitive.h>
#include <xsi_kinematicstate.h>
#include <xsi_kinematics.h>
#include <xsi_port.h>
#include <xsi_operator.h>
#include <xsi_customoperator.h>
#include <xsi_operatorcontext.h>
#include <xsi_point.h>
#include <xsi_nurbsdata.h>
#include <xsi_polygonmesh.h>
#include <xsi_facet.h>

// Defines port, group and map identifiers used for registering the ICENode
enum IDs
{
   ID_IN_StrandPosition = 0,
   ID_IN_Count = 1,
   ID_IN_Min = 2,
   ID_IN_Max = 3,
   ID_G_100 = 100,
   ID_OUT_Result = 200,
   ID_G_300 = 300,
   ID_TYPE_CNS = 400,
   ID_STRUCT_CNS,
   ID_CTXT_CNS,
   ID_UNDEF = ULONG_MAX
};

// Defines port, group and map identifiers used for registering the ICENode
enum SIM_IDs
{
   SIM_ID_IN_In_RootPosition = 1,
   SIM_ID_IN_In_StrandPosition = 2,
   SIM_ID_IN_In_StrandVelocity = 3,
   SIM_ID_IN_In_StrandLength = 4,
   SIM_ID_G_100 = 100,
   SIM_ID_OUT_Out_StrandPosition = 200,
   SIM_ID_G_300 = 300,
   SIM_ID_TYPE_CNS = 400,
   SIM_ID_STRUCT_CNS,
   SIM_ID_CTXT_CNS,
   SIM_ID_UNDEF = ULONG_MAX
};

enum Array_IDs
{
   Array_ID_IN_Vector = 0,
   Array_ID_G_100 = 100,
   Array_ID_OUT_Result = 200,
   Array_ID_G_300 = 300,
   Array_ID_TYPE_CNS = 400,
   Array_ID_STRUCT_CNS,
   Array_ID_CTXT_CNS,
   Array_ID_UNDEF = ULONG_MAX
};
enum Lattice_IDs
{
   Lattice_ID_IN_Point = 0,
   Lattice_ID_IN_Subdivision = 1,
   Lattice_ID_IN_Step = 2,
   Lattice_ID_IN_Reference = 3,
   Lattice_ID_IN_Current = 4,
   Lattice_ID_G_100 = 100,
   Lattice_ID_OUT_Result = 200,
   Lattice_ID_G_300 = 300,
   Lattice_ID_TYPE_CNS = 400,
   Lattice_ID_STRUCT_CNS,
   Lattice_ID_CTXT_CNS,
   Lattice_ID_UNDEF = ULONG_MAX
};

XSI::CStatus Register_nest_StrandFitting( XSI::PluginRegistrar& in_reg );
XSI::CStatus Register_nest_HairSolver_Node( XSI::PluginRegistrar& in_reg );
XSI::CStatus Register_nest_ArrayNode( XSI::PluginRegistrar& in_reg );
XSI::CStatus Register_nest_LatticeNode( XSI::PluginRegistrar& in_reg );

using namespace XSI; 
using namespace MATH;

XSIPLUGINCALLBACK CStatus XSILoadPlugin( PluginRegistrar& in_reg )
{
   in_reg.PutAuthor(L"hmathee");
   in_reg.PutName(L"nest_melena Plugin");
   in_reg.PutEmail(L"");
   in_reg.PutURL(L"");
   in_reg.PutVersion(2,4);

   Register_nest_StrandFitting( in_reg );
   Register_nest_HairSolver_Node( in_reg );
   Register_nest_ArrayNode( in_reg );
   Register_nest_LatticeNode( in_reg );
   in_reg.RegisterOperator(L"nest_ice_StrandExtrude");
   in_reg.RegisterCommand(L"apply_nest_ice_StrandExtrude",L"apply_nest_ice_StrandExtrude");
   in_reg.RegisterMenu(siMenuTbGetPrimitiveID,L"nest_create_ICE_Strand_Extrusion",false,false);
   in_reg.RegisterOperator(L"nest_hair2curves");
   in_reg.RegisterCommand(L"apply_nest_hair2curves",L"apply_nest_hair2curves");
   in_reg.RegisterOperator(L"nest_strands2hair");
   in_reg.RegisterCommand(L"apply_nest_strands2hair",L"apply_nest_strands2hair");

   //RegistrationInsertionPoint - do not remove this line

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus XSIUnloadPlugin( const PluginRegistrar& in_reg )
{
   CString strPluginName;
   strPluginName = in_reg.GetName();
   Application().LogMessage(strPluginName + L" has been unloaded.",siVerboseMsg);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus apply_nest_ice_StrandExtrude_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Command oCmd;
   oCmd = ctxt.GetSource();
   oCmd.PutDescription(L"Create an instance of nest_ice_StrandExtrude operator");
   oCmd.SetFlag(siNoLogging,false);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus apply_nest_ice_StrandExtrude_Execute( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Application().LogMessage(L"apply_nest_ice_StrandExtrude_Execute called",siVerboseMsg);

   Selection l_pSelection = Application().GetSelection();
   CRef l_pCloud;
   bool l_bHaveCloud = false;

   // search the selection for a mesh and a pointcloud
   for(long i=0;i<l_pSelection.GetCount();i++)
   {
      X3DObject l_pSelObj(l_pSelection.GetItem(i));
      if(l_pSelObj.GetType().IsEqualNoCase(L"pointcloud"))
      {
         l_pCloud = l_pSelObj.GetActivePrimitive().GetRef();
         l_bHaveCloud = true;
         break;
      }
   }

   // if we are missing the pointcloud, error out
   if( ! l_bHaveCloud )
   {
      Application().LogMessage(L"Please select one pointcloud!",XSI::siErrorMsg);
      return CStatus::Fail;
   }

   // create an empty polygonmesh!
   CValueArray cmdArgs;
   CValue returnVal;
   cmdArgs.Resize(1);
   cmdArgs[0] = L"EmptyPolygonMesh";
   Application().ExecuteCommand(L"SIGetPrim",cmdArgs,returnVal);
   CValueArray returnVals = returnVal;
   CRef outputMesh = (CRef)returnVals[0];

   // name the output mesh according to the source object
   X3DObject(Primitive(outputMesh).GetParent()).PutName(X3DObject(Primitive(outputMesh).GetParent()).GetName()+L"_strands");

   // create the operator
   CustomOperator newOp = Application().GetFactory().CreateObject(L"nest_ice_StrandExtrude");
   newOp.AddOutputPort(outputMesh);
   newOp.AddInputPort(l_pCloud);
   newOp.Connect();
   ctxt.PutAttribute( L"ReturnValue", newOp.GetRef() );

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus nest_ice_StrandExtrude_Define( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CustomOperator oCustomOperator;
   Parameter oParam;
   CRef oPDef;
   Factory oFactory = Application().GetFactory();
   oCustomOperator = ctxt.GetSource();

   oPDef = oFactory.CreateParamDef(L"subdiv",CValue::siInt4,siAnimatable|siPersistable,L"subdiv",L"subdiv",4,2,100,3,12);
   oCustomOperator.AddParameter(oPDef,oParam);
   oPDef = oFactory.CreateParamDef(L"size",CValue::siFloat,siAnimatable|siPersistable,L"size",L"size",1,-1000,1000,0.001,10);
   oCustomOperator.AddParameter(oPDef,oParam);

   oCustomOperator.PutAlwaysEvaluate(false);
   oCustomOperator.PutDebug(0);
   return CStatus::OK;
}

CStatus Register_nest_StrandFitting( PluginRegistrar& in_reg )
{
   ICENodeDef nodeDef;
   nodeDef = Application().GetFactory().CreateICENodeDef(L"nest_StrandFitting");

   CStatus st;

   // Add input ports and groups.
   st = nodeDef.AddPortGroup(ID_G_100);
   st.AssertSucceeded( ) ;

   st = nodeDef.AddInputPort(ID_IN_StrandPosition,ID_G_100,siICENodeDataVector3,siICENodeStructureArray,siICENodeContextAny,L"StrandPosition",L"StrandPosition",CVector3f(1.0,1.0,1.0),ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
   st.AssertSucceeded( ) ;

   st = nodeDef.AddInputPort(ID_IN_Count,ID_G_100,siICENodeDataLong,siICENodeStructureSingle,siICENodeContextAny,L"Count",L"Count",5,ID_UNDEF,ID_STRUCT_CNS,ID_CTXT_CNS);
   st.AssertSucceeded( ) ;

   st = nodeDef.AddInputPort(ID_IN_Min,ID_G_100,siICENodeDataFloat,siICENodeStructureSingle,siICENodeContextAny,L"Min",L"Min",0.0f,ID_UNDEF,ID_STRUCT_CNS,ID_CTXT_CNS);
   st.AssertSucceeded( ) ;

   st = nodeDef.AddInputPort(ID_IN_Max,ID_G_100,siICENodeDataFloat,siICENodeStructureSingle,siICENodeContextAny,L"Max",L"Max",1.0f,ID_UNDEF,ID_STRUCT_CNS,ID_CTXT_CNS);
   st.AssertSucceeded( ) ;

   // Add output ports and groups.
   st = nodeDef.AddPortGroup(ID_G_300);
   st.AssertSucceeded( ) ;

   st = nodeDef.AddOutputPort(ID_OUT_Result,ID_G_300,siICENodeDataVector3,siICENodeStructureArray,siICENodeContextAny,L"Result",L"Result",ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
   st.AssertSucceeded( ) ;

   PluginItem nodeItem = in_reg.RegisterICENode(nodeDef);
   nodeItem.PutCategories(L"Array");

   return CStatus::OK;
}

CStatus Register_nest_HairSolver_Node( PluginRegistrar& in_reg )
{
   ICENodeDef nodeDef;
   nodeDef = Application().GetFactory().CreateICENodeDef(L"nest_HairSolver");

   CStatus st;

   // Add input ports and groups.
   st = nodeDef.AddPortGroup(SIM_ID_G_100);
   st.AssertSucceeded( ) ;

   st = nodeDef.AddInputPort(SIM_ID_IN_In_RootPosition,SIM_ID_G_100,siICENodeDataVector3,siICENodeStructureSingle,siICENodeContextAny,L"In_RootPosition",L"In_RootPosition",CVector3f(1.0,1.0,1.0),SIM_ID_UNDEF,SIM_ID_UNDEF,SIM_ID_CTXT_CNS);
   st.AssertSucceeded( ) ;

   st = nodeDef.AddInputPort(SIM_ID_IN_In_StrandPosition,SIM_ID_G_100,siICENodeDataVector3,siICENodeStructureArray,siICENodeContextAny,L"In_StrandPosition",L"In_StrandPosition",CVector3f(1.0,1.0,1.0),SIM_ID_UNDEF,SIM_ID_UNDEF,SIM_ID_CTXT_CNS);
   st.AssertSucceeded( ) ;

   st = nodeDef.AddInputPort(SIM_ID_IN_In_StrandVelocity,SIM_ID_G_100,siICENodeDataVector3,siICENodeStructureArray,siICENodeContextAny,L"In_StrandVelocity",L"In_StrandVelocity",CVector3f(1.0,1.0,1.0),SIM_ID_UNDEF,SIM_ID_UNDEF,SIM_ID_CTXT_CNS);
   st.AssertSucceeded( ) ;

   st = nodeDef.AddInputPort(SIM_ID_IN_In_StrandLength,SIM_ID_G_100,siICENodeDataFloat,siICENodeStructureSingle,siICENodeContextAny,L"In_StrandLength",L"In_StrandLength",1,SIM_ID_UNDEF,SIM_ID_UNDEF,SIM_ID_CTXT_CNS);
   st.AssertSucceeded( ) ;

   // Add output ports and groups.
   st = nodeDef.AddPortGroup(SIM_ID_G_300);
   st.AssertSucceeded( ) ;

   st = nodeDef.AddOutputPort(SIM_ID_OUT_Out_StrandPosition,SIM_ID_G_300,siICENodeDataVector3,siICENodeStructureArray,siICENodeContextAny,L"Out_StrandPosition",L"Out_StrandPosition",SIM_ID_UNDEF,SIM_ID_UNDEF,SIM_ID_CTXT_CNS);
   st.AssertSucceeded( ) ;

   PluginItem nodeItem = in_reg.RegisterICENode(nodeDef);
   nodeItem.PutCategories(L"Simulation");

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus nest_StrandFitting_Evaluate( ICENodeContext& in_ctxt )
{
   // The current output port being evaluated...
   ULONG out_portID = in_ctxt.GetEvaluatedOutputPortID( );

   switch( out_portID )
   {
      case ID_OUT_Result:
      {
         // Get the output port array ...
         CDataArray2DVector3f outData( in_ctxt );

         // Get the input data buffers for each port
         CDataArray2DVector3f StrandPositionData( in_ctxt, ID_IN_StrandPosition );
         CDataArrayLong CountData( in_ctxt, ID_IN_Count );
         CDataArrayFloat MinData( in_ctxt, ID_IN_Min );
         CDataArrayFloat MaxData( in_ctxt, ID_IN_Max );

         // We need a CIndexSet to iterate over the data
         CIndexSet indexSet( in_ctxt );
         for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
         {
            CDataArray2DVector3f::Accessor StrandPos = StrandPositionData[it];

            // make sure to tackle negative counts
            if(CountData[it] <= 0 || StrandPos.GetCount() <= 1)
            {
               outData.Resize(it,0);
               continue;
            }

            // clamp the range
            if(MinData[it] < 0)MinData[it] = 0;
            if(MaxData[it] < 0)MaxData[it] = 0;
            if(MinData[it] > 1)MinData[it] = 1;
            if(MaxData[it] > 1)MaxData[it] = 1;
            if(MinData[it] > MaxData[it])
            {
               float temp = MinData[it];
               MinData[it] = MaxData[it];
               MaxData[it] = temp;
            }


            // measure the length of the strand
            CDoubleArray StrandLength (StrandPos.GetCount()-1);
            CDoubleArray StrandLengthAcc (StrandPos.GetCount()-1);
            double StrandLengthSum = 0;
            for(long i=0;i<StrandLength.GetCount();i++)
            {
               CVector3f Segment;
               Segment.Sub(StrandPos[i+1],StrandPos[i]);
               StrandLength[i] = Segment.GetLength();
               StrandLengthSum += StrandLength[i];
               StrandLengthAcc[i] = StrandLengthSum;
            }

            double SegmentLength = (MaxData[it]-MinData[it]) * StrandLengthSum / float(CountData[it]-1);

            // find the first index to start with...
            double currentLength = 0;
            long firstIndex = 0;
            while(currentLength < MinData[it] * StrandLengthSum)
            {
               currentLength += StrandLength[firstIndex];
               firstIndex++;
            }
            if(currentLength > 0)
               currentLength -= StrandLength[firstIndex];
            if(firstIndex>0)
               firstIndex--;

            // Set the output data buffer, etc...
            outData.Resize(it,CountData[it]);

            // walk the strand and place the new points
            for(long i=0;i<CountData[it];i++)
            {
               double acc = (firstIndex == 0) ? 0.0f : StrandLengthAcc[firstIndex-1];
               double ratio = (currentLength - acc) / StrandLength[firstIndex];
               if(ratio > 1)ratio = 1;
               if(ratio < 0)ratio = 0;
               outData[it][i].LinearlyInterpolate(StrandPos[firstIndex],StrandPos[firstIndex+1],(float)ratio);

               currentLength += SegmentLength;
               acc = StrandLengthAcc[firstIndex];
               while(currentLength > acc && firstIndex < (long)StrandPos.GetCount() - 2)
               {
                  acc = StrandLengthAcc[++firstIndex];
               }
            }
         }
      }
      break;

      // Other output ports...

   };

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus nest_create_ICE_Strand_Extrusion_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Menu oMenu;
   oMenu = ctxt.GetSource();
   MenuItem oNewItem;
   oMenu.AddCommandItem(L"Create ICE Strand Extrusion",L"apply_nest_ice_StrandExtrude",oNewItem);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus nest_ice_StrandExtrude_Update( CRef& in_ctxt )
{
   OperatorContext ctxt( in_ctxt );
   Geometry cloudGeo = Primitive(ctxt.GetInputValue(0)).GetGeometry();

   // get all ice attributes
   ICEAttribute PositionAttr = cloudGeo.GetICEAttributeFromName(L"StrandPosition");
   ICEAttribute OrientationAttr = cloudGeo.GetICEAttributeFromName(L"StrandRotation");
   ICEAttribute SizeAttr = cloudGeo.GetICEAttributeFromName(L"StrandSize");
   ICEAttribute ScaleAttr = cloudGeo.GetICEAttributeFromName(L"StrandScale");

   // check some attributes for validity
   if( !PositionAttr.IsValid() || !PositionAttr.IsDefined() )
   {
      Application().LogMessage(L"nest_ice_StrandExtrude: The used pointcloud doesn't have the 'StrandPosition' attribute defined.",XSI::siErrorMsg);
      return CStatus::OK;
   }
   if( !OrientationAttr.IsValid())
   {
      Application().LogMessage(L"nest_ice_StrandExtrude: The used pointcloud doesn't have the 'StrandRotation' attribute defined.",XSI::siErrorMsg);
      Application().LogMessage(L"nest_ice_StrandExtrude: Don't forget to use the 'Prepare Strands for Extrusion' compound!",XSI::siVerboseMsg);
      return CStatus::OK;
   }
   if( !SizeAttr.IsValid())
   {
      Application().LogMessage(L"nest_ice_StrandExtrude: The used pointcloud doesn't have the 'StrandSize' attribute defined.",XSI::siErrorMsg);
      return CStatus::OK;
   }
   if( !ScaleAttr.IsValid())
   {
      Application().LogMessage(L"nest_ice_StrandExtrude: The used pointcloud doesn't have the 'StrandScale' attribute defined.",XSI::siErrorMsg);
      return CStatus::OK;
   }

   LONG subdiv = ctxt.GetParameterValue(L"subdiv");
   float factor = ctxt.GetParameterValue(L"size");

   // get the number of particles
   ULONG l_ulCount = PositionAttr.GetElementCount();

   // create the output arrays
   CVector3Array outputPos;
   CLongArray outputPoly;

   // create the points to project, 
   // simply rotate the x vector around y accordingly
   CVector3Array pointsToInstance(subdiv);
   pointsToInstance[0].Set(1,0,0);
   CRotation pointsToInstanceRot(0,-.5 * DegreesToRadians(360.0 / float(subdiv)),0);
   pointsToInstance[0].MulByMatrix3InPlace(pointsToInstanceRot.GetMatrix());
   pointsToInstanceRot.SetFromXYZAngles(0,DegreesToRadians(360.0 / float(subdiv)),0);
   for(long i=1;i<subdiv;i++)
      pointsToInstance[i].MulByMatrix3(pointsToInstance[i-1],pointsToInstanceRot.GetMatrix());

   // create arrays to store the ice attribute data
   CICEAttributeDataArray2D<CVector3f> Position2D;
   CICEAttributeDataArray2D<CVector3f> Orientation2D;
   CICEAttributeDataArray2D<CVector3f> Scale2D;
   CICEAttributeDataArray2D<float> Size2D;
   CICEAttributeDataArray<CVector3f> Position;
   CICEAttributeDataArray<CVector3f> Orientation;
   CICEAttributeDataArray<CVector3f> Scale;
   CICEAttributeDataArray<float> Size;

   CTransformation xf;
   CVector3 vec;

   // access the actual ice attribute data
   PositionAttr.GetDataArray2D(Position2D);
   OrientationAttr.GetDataArray2D(Orientation2D);
   ScaleAttr.GetDataArray2D(Scale2D);
   SizeAttr.GetDataArray2D(Size2D);
   bool firstCap = false;
   bool subdiv2 = subdiv == 2;
   
   for(ULONG pointIndex = 0; pointIndex < l_ulCount; pointIndex ++ )
   {
      Position2D.GetSubArray(pointIndex,Position);
      Orientation2D.GetSubArray(pointIndex,Orientation);
      Scale2D.GetSubArray(pointIndex,Scale);
      Size2D.GetSubArray(pointIndex,Size);

      if(Position.GetCount() > 1)
      {
         for(ULONG i=0;i<Position.GetCount();i++)
         {
            xf.SetPosX(Position[i].GetX());
            xf.SetPosY(Position[i].GetY());
            xf.SetPosZ(Position[i].GetZ());

            xf.SetRotationFromXYZAnglesValues(
               DegreesToRadians(Orientation[i].GetX()),
               DegreesToRadians(Orientation[i].GetY()),
               DegreesToRadians(Orientation[i].GetZ()));

            xf.SetSclX(factor * Size[i] * Scale[i].GetX());
            xf.SetSclY(factor * Size[i] * Scale[i].GetY());
            xf.SetSclZ(factor * Size[i] * Scale[i].GetZ());

            firstCap = (i==0);

            if(subdiv2 && !firstCap)
            {
               outputPoly.Add(4);
               outputPoly.Add(outputPos.GetCount());
               outputPoly.Add(outputPos.GetCount()+1);
               outputPoly.Add(outputPos.GetCount()-1);
               outputPoly.Add(outputPos.GetCount()-2);
            }
            else if(firstCap)
            {
               // add the cap
               outputPoly.Add(subdiv);
               for(long j=0;j<subdiv;j++)
                  outputPoly.Add(outputPos.GetCount()+j);
            }

            for(long j=0;j<subdiv;j++)
            {
               if(!firstCap && !subdiv2)
               {
                  outputPoly.Add(4);
                  outputPoly.Add(outputPos.GetCount() - subdiv);
                  outputPoly.Add(outputPos.GetCount());
                  if(j==subdiv-1)
                  {
                     outputPoly.Add(outputPos.GetCount() - subdiv + 1);
                     outputPoly.Add(outputPos.GetCount()- subdiv - subdiv + 1);
                  }
                  else
                  {
                     outputPoly.Add(outputPos.GetCount() + 1);
                     outputPoly.Add(outputPos.GetCount() - subdiv + 1);
                  }
               }

               vec.MulByTransformation(pointsToInstance[j],xf);
               outputPos.Add(vec);
            }

         }

         // add the cap
         if(!subdiv2)
         {
            outputPoly.Add(subdiv);
            for(long j=0;j<subdiv;j++)
               outputPoly.Add(outputPos.GetCount()-1-j);
         }
      }
   }

   Primitive output = ctxt.GetOutputTarget();
   PolygonMesh(output.GetGeometry()).Set(outputPos,outputPoly);

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus apply_nest_hair2curves_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Command oCmd;
   oCmd = ctxt.GetSource();
   oCmd.PutDescription(L"Create an instance of nest_hair2curves operator");
   oCmd.SetFlag(siNoLogging,false);
   return CStatus::OK;
}
XSIPLUGINCALLBACK CStatus apply_nest_strands2hair_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Command oCmd;
   oCmd = ctxt.GetSource();
   oCmd.PutDescription(L"Create an instance of nest_hair2curves operator");
   oCmd.SetFlag(siNoLogging,false);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus apply_nest_hair2curves_Execute( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CValueArray args = ctxt.GetAttribute(L"Arguments");

   Selection sel = Application().GetSelection();
   CRef hairPrimRef;
   for(long i=0;i<sel.GetCount();i++)
   {
      if(SIObject(sel.GetItem(i)).GetType().IsEqualNoCase(L"hair"))
      {
         hairPrimRef = X3DObject(sel.GetItem(i)).GetActivePrimitive();
      }
   }

   if(hairPrimRef.GetAsText().IsEmpty())
   {
      Application().LogMessage(L"Please select one hair object!",siErrorMsg);
      return CStatus::Abort;
   }

   CRef hairKineRef;
   hairKineRef = Primitive(hairPrimRef).GetParent3DObject().GetKinematics().GetGlobal();

   CRef hairGenOpRef;
   hairGenOpRef.Set(hairPrimRef.GetAsText()+L".hairGenOp");
   if(hairGenOpRef.GetAsText().IsEmpty())
   {
      Application().LogMessage(L"Fatal error: The hair doesn't have a hairGenOperator!",siErrorMsg);
      return CStatus::Abort;
   }

   Operator hairGenOp(hairGenOpRef);
   CRef emitterPrimRef,emitterKineRef,emitterClusterRef;
   emitterPrimRef = Port(hairGenOp.GetInputPorts().GetItem(0)).GetTarget();
   emitterKineRef = Port(hairGenOp.GetInputPorts().GetItem(1)).GetTarget();
   emitterClusterRef = Port(hairGenOp.GetInputPorts().GetItem(3)).GetTarget();

   if(!SIObject(emitterPrimRef).GetType().IsEqualNoCase(L"polymsh")
   || !SIObject(emitterClusterRef).GetType().IsEqualNoCase(L"poly"))
   {
      Application().LogMessage(L"Error: The hair needs to be grown of a polygon mesh cluster!",siErrorMsg);
      return CStatus::Abort;
   }

   // now create the curves primitive!
   CValueArray cmdArgs;
   CValue returnVal;
   Application().ExecuteCommand(L"SICreateCurve",cmdArgs,returnVal);
   CRef curvePrimRef = X3DObject(CRef(returnVal)).GetActivePrimitive();

   /*
   cmdArgs.Resize(2);
   cmdArgs[0] = Primitive(hairPrimRef).GetParent3DObject().GetFullName();
   cmdArgs[1] = Primitive(curvePrimRef).GetParent3DObject().GetFullName();
   Application().ExecuteCommand(L"ParentObj",cmdArgs,returnVal);
   */
   Primitive(curvePrimRef).GetParent3DObject().PutName(
      Primitive(hairPrimRef).GetParent3DObject().GetName()+L"_curves");

   // now create the op!
   CustomOperator newOp = Application().GetFactory().CreateObject(L"nest_hair2curves");
   newOp.AddOutputPort(curvePrimRef);
   newOp.AddInputPort(hairPrimRef);
   newOp.AddInputPort(hairKineRef);
   newOp.AddInputPort(emitterPrimRef);
   newOp.AddInputPort(emitterKineRef);
   newOp.AddInputPort(emitterClusterRef);
   newOp.Connect();
   ctxt.PutAttribute( L"ReturnValue", newOp.GetRef() );

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus apply_nest_strands2hair_Execute( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CValueArray args = ctxt.GetAttribute(L"Arguments");

   Selection sel = Application().GetSelection();
   CRef hairPrimRef;
   CRef pointPrimRef;
   for(long i=0;i<sel.GetCount();i++)
   {
      if(SIObject(sel.GetItem(i)).GetType().IsEqualNoCase(L"hair"))
      {
         hairPrimRef = X3DObject(sel.GetItem(i)).GetActivePrimitive();
      }
      else if(SIObject(sel.GetItem(i)).GetType().IsEqualNoCase(L"pointcloud"))
      {
         pointPrimRef = X3DObject(sel.GetItem(i)).GetActivePrimitive();
      }
   }

   if(hairPrimRef.GetAsText().IsEmpty() || pointPrimRef.GetAsText().IsEmpty())
   {
      Application().LogMessage(L"Please select one hair object and one particle cloud!",siErrorMsg);
      return CStatus::Abort;
   }

   CRef hairKineRef;
   hairKineRef = Primitive(hairPrimRef).GetParent3DObject().GetKinematics().GetGlobal();

   // now create the op!
   CustomOperator newOp = Application().GetFactory().CreateObject(L"nest_strands2hair");
   newOp.AddOutputPort(hairPrimRef);
   newOp.AddInputPort(hairKineRef);
   newOp.AddInputPort(pointPrimRef);
   newOp.AddInputPort(hairPrimRef);
   newOp.Connect();
   ctxt.PutAttribute( L"ReturnValue", newOp.GetRef() );

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus nest_hair2curves_Define( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CustomOperator oCustomOperator;
   Parameter oParam;
   CRef oPDef;
   Factory oFactory = Application().GetFactory();
   oCustomOperator = ctxt.GetSource();

   oPDef = oFactory.CreateParamDef(L"mode",CValue::siInt4,siPersistable | siAnimatable,L"",L"",0,0,100,0,1);
   oCustomOperator.AddParameter(oPDef,oParam);
   oPDef = oFactory.CreateParamDef(L"count",CValue::siInt4,siPersistable | siAnimatable,L"",L"",1000,0,1000000,0,10000);
   oCustomOperator.AddParameter(oPDef,oParam);

   oCustomOperator.PutAlwaysEvaluate(true);
   oCustomOperator.PutDebug(0);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus nest_strands2hair_Define( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CustomOperator oCustomOperator;
   Parameter oParam;
   CRef oPDef;
   Factory oFactory = Application().GetFactory();
   oCustomOperator = ctxt.GetSource();

   oCustomOperator.PutAlwaysEvaluate(true);
   oCustomOperator.PutDebug(0);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus nest_hair2curves_DefineLayout( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   PPGLayout oLayout;
   PPGItem oItem;
   oLayout = ctxt.GetSource();
   oLayout.Clear();
   CValueArray modeItems(6);
   modeItems[0] = L"Guide Hairs";
   modeItems[1] = (LONG)0l;
   modeItems[2] = L"Render Hairs (Local)";
   modeItems[3] = (LONG)1l;
   modeItems[4] = L"Render Hairs (Global)";
   modeItems[5] = (LONG)2l;
   oLayout.AddEnumControl(L"mode",modeItems,L"Mode");
   oLayout.AddItem(L"count",L"Render Hair Count");
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus nest_hair2curves_Update( CRef& in_ctxt )
{
   OperatorContext ctxt( in_ctxt );
   HairPrimitive hairPrim(ctxt.GetInputValue(0));
   Geometry hairGeo = hairPrim.GetGeometry();
   CVector3Array hairPos = hairGeo.GetPoints().GetPositionArray();
   KinematicState hairKine(ctxt.GetInputValue(1));
   CTransformation hairXF = hairKine.GetTransform();
   Primitive emitterPrim(ctxt.GetInputValue(2));
   PolygonMesh emitterGeo = emitterPrim.GetGeometry();
   KinematicState emitterKine(ctxt.GetInputValue(3));
   CTransformation emitterXF = emitterKine.GetTransform();
   Cluster emitterCluster(ctxt.GetInputValue(4));
   CLongArray clusterElements = emitterCluster.GetElements().GetArray();

   LONG mode = (LONG)ctxt.GetParameterValue(L"mode");
   CNurbsCurveDataArray curveData;
   
   if(mode == 0) // guide hairs
   {
      // collect all points for the roots of the hairs!
      CLongArray emitterPntUsed(emitterGeo.GetPoints().GetCount());
      CLongArray emitterPntIndex;
      CLongArray facetIndex;
      for(long i=0;i<clusterElements.GetCount();i++)
      {
         facetIndex = Facet(emitterGeo.GetFacets().GetItem(clusterElements[i])).GetPoints().GetIndexArray();
         for(long j=0;j<facetIndex.GetCount();j++)
         {
            if(emitterPntUsed[facetIndex[j]]==0)
            {
               emitterPntUsed[facetIndex[j]] = 1;
               emitterPntIndex.Add(facetIndex[j]);
            }
         }
      }
   
      CDoubleArray knots(15);
      for(long i=0;i<15;i++)
         knots[i] = i;
   
      // ok, now we know we can build the curves!
      long curveCount = emitterPntIndex.GetCount();
      curveData = CNurbsCurveDataArray(curveCount);
      long hairOffset = 0;
      for(long i=0;i<curveCount;i++)
      {
         curveData[i].m_bClosed = false;
         curveData[i].m_lDegree = 1;
         curveData[i].m_siParameterization = siUniformParameterization;
         curveData[i].m_aKnots = knots;
         curveData[i].m_aControlPoints.Resize(15);
   
         // set the root
         CVector3 pos = MapObjectPositionToWorldSpace(emitterXF,Point(emitterGeo.GetPoints().GetItem(emitterPntIndex[i])).GetPosition());
         curveData[i].m_aControlPoints[0].Set(pos.GetX(),pos.GetY(),pos.GetZ(),1.0);
   
         for(long j=1;j<15;j++)
         {
            hairPos[hairOffset] = MapObjectPositionToWorldSpace(hairXF,hairPos[hairOffset]);
            curveData[i].m_aControlPoints[j].Set(hairPos[hairOffset].GetX(),hairPos[hairOffset].GetY(),hairPos[hairOffset].GetZ(),1.0);
            hairOffset++;
         }
      }
   }
   else if(mode == 1)
   {
      LONG curveCount = (LONG)ctxt.GetParameterValue(L"count");
      curveData = CNurbsCurveDataArray(curveCount);
      CRenderHairAccessor accessor(hairPrim.GetRenderHairAccessor(curveCount));
      
      CLongArray vertexCount;
      CFloatArray vertexPos;
      
      CDoubleArray knots;
      LONG lastVertexCount = 0;
      LONG currVertexCount = 0;
		LONG curveDataI = 0;
      LONG offset = 0;
      
      while(accessor.Next())
      {
         accessor.GetVerticesCount(vertexCount);
         accessor.GetVertexPositions(vertexPos);
         offset = 0;
         
         LONG chunkCount = vertexCount.GetCount();
         for(LONG i=0;i<chunkCount;i++)
         {
            currVertexCount = vertexCount[i];
            if(lastVertexCount != currVertexCount)
            {
               knots.Resize(currVertexCount);
               for(long i=0;i<currVertexCount;i++)
                  knots[i] = i;
               lastVertexCount = currVertexCount;
            }
            
            curveData[curveDataI].m_bClosed = false;
            curveData[curveDataI].m_lDegree = 1;
            curveData[curveDataI].m_siParameterization = siUniformParameterization;
            curveData[curveDataI].m_aKnots = knots;
            curveData[curveDataI].m_aControlPoints.Resize(currVertexCount);
      
            for(long j=0;j<currVertexCount;j++)
            {
               curveData[curveDataI].m_aControlPoints[j].Set(vertexPos[offset],vertexPos[offset+1],vertexPos[offset+2],1.0);
               offset+=3;
            }
				curveDataI++;
         }
      }
   }
   else if(mode == 2)
   {
      LONG curveCount = (LONG)ctxt.GetParameterValue(L"count");
      curveData = CNurbsCurveDataArray(curveCount);
      CRenderHairAccessor accessor(hairPrim.GetRenderHairAccessor(curveCount));

      CLongArray vertexCount;
      CFloatArray vertexPos;
      CVector3 singlePos;

      CDoubleArray knots;
      LONG lastVertexCount = 0;
      LONG currVertexCount = 0;
		LONG curveDataI = 0;
      LONG offset = 0;

      while(accessor.Next())
      {
         accessor.GetVerticesCount(vertexCount);
         accessor.GetVertexPositions(vertexPos);
         offset = 0;

         LONG chunkCount = vertexCount.GetCount();
         for(LONG i=0;i<chunkCount;i++)
         {
            currVertexCount = vertexCount[i];
            if(lastVertexCount != currVertexCount)
            {
               knots.Resize(currVertexCount);
               for(long i=0;i<currVertexCount;i++)
                  knots[i] = i;
               lastVertexCount = currVertexCount;
            }

            curveData[curveDataI].m_bClosed = false;
            curveData[curveDataI].m_lDegree = 1;
            curveData[curveDataI].m_siParameterization = siUniformParameterization;
            curveData[curveDataI].m_aKnots = knots;
            curveData[curveDataI].m_aControlPoints.Resize(currVertexCount);

            for(long j=0;j<currVertexCount;j++)
            {
               singlePos.Set(vertexPos[offset],vertexPos[offset+1],vertexPos[offset+2]);
               singlePos = MapObjectPositionToWorldSpace(hairXF,singlePos);
               curveData[curveDataI].m_aControlPoints[j].Set(singlePos.GetX(),singlePos.GetY(),singlePos.GetZ(),1.0);
               offset+=3;
            }
				curveDataI++;
         }
      }
   }

   Primitive outPrim = ctxt.GetOutputTarget();
   NurbsCurveList outGeo = outPrim.GetGeometry();
   outGeo.Set(curveData);

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus nest_strands2hair_Update( CRef& in_ctxt )
{
   OperatorContext ctxt( in_ctxt );
   KinematicState hairKine(ctxt.GetInputValue(0));
   CTransformation hairXF = hairKine.GetTransform();
   Primitive pointPrim(ctxt.GetInputValue(1));
   Geometry pointGeo = pointPrim.GetGeometry();
   Primitive hairPrim(ctxt.GetInputValue(2));

   ICEAttribute StrandAttr = pointGeo.GetICEAttributeFromName(L"StrandPosition");
   CICEAttributeDataArray2DVector3f strandPos2D;
   StrandAttr.GetDataArray2D(strandPos2D);
   CICEAttributeDataArrayVector3f strandPos;

   long hairCount = StrandAttr.GetElementCount();
   CVector3Array hairPos(hairCount * 14);
   long hairOffset = 0;
   for(long i=0;i<hairCount;i++)
   {
      strandPos2D.GetSubArray(i,strandPos);
      for(long j=1;j<15;j++)
      {
         hairPos[hairOffset].Set(strandPos[j].GetX(),strandPos[j].GetY(),strandPos[j].GetZ());
         hairPos[hairOffset] = MapWorldPositionToObjectSpace(hairXF,hairPos[hairOffset]);
         hairOffset++;
      }
   }

   Primitive outPrim = ctxt.GetOutputTarget();
   Geometry outGeo = outPrim.GetGeometry();
   outGeo.GetPoints().PutPositionArray(hairPos);

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus nest_HairSolver_Evaluate( ICENodeContext& in_ctxt )
{
   // The current output port being evaluated...
   ULONG out_portID = in_ctxt.GetEvaluatedOutputPortID( );

   switch( out_portID )
   {
      case SIM_ID_OUT_Out_StrandPosition:
      {
         // Get the output port array ...
         CDataArray2DVector3f outData( in_ctxt );

         // Get the input data buffers for each port
         CDataArrayVector3f In_RootPositionData( in_ctxt, SIM_ID_IN_In_RootPosition );
         CDataArray2DVector3f In_StrandPositionData( in_ctxt, SIM_ID_IN_In_StrandPosition );
         CDataArray2DVector3f In_StrandVelocityData( in_ctxt, SIM_ID_IN_In_StrandVelocity );
         CDataArrayFloat In_StrandLengthData( in_ctxt, SIM_ID_IN_In_StrandLength );

         // We need a CIndexSet to iterate over the data
         CIndexSet indexSet( in_ctxt );

         //Application().LogMessage( L" test test test ");
         for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
         {
            // Set the output data buffer, etc...
            CDataArray2DVector3f::Accessor In_StrandPositionDataSubArray = In_StrandPositionData[it];
            CDataArray2DVector3f::Accessor In_StrandVelocityDataSubArray = In_StrandVelocityData[it];
            ULONG l_ulCount = In_StrandPositionDataSubArray.GetCount();

            CDataArray2DVector3f::Accessor outDataSubArray = outData.Resize(it,l_ulCount);

            CVector3f l_vPreviousPos = In_RootPositionData[it];
            float l_fLength = In_StrandLengthData[it];
            l_fLength /= (float)(l_ulCount-1);
            outDataSubArray[0] = l_vPreviousPos;

            //Application().LogMessage(CValue(l_ulCount).GetAsText());
            for(ULONG i=1;i<l_ulCount;i++)
            {
               CVector3f l_vPos;
               l_vPos.Add(In_StrandPositionDataSubArray[i],In_StrandVelocityDataSubArray[i]);

               l_vPos.Sub(l_vPos,l_vPreviousPos);
               l_vPos.NormalizeInPlace();
               l_vPos.ScaleInPlace(l_fLength);
               l_vPos.AddInPlace(l_vPreviousPos);

               outDataSubArray[i] = l_vPos;
               l_vPreviousPos = l_vPos;
            }
         }
      }
      break;

      // Other output ports...

   };

   return CStatus::OK;
}

CStatus Register_nest_ArrayNode( PluginRegistrar& in_reg )
{
   ICENodeDef nodeDef;
   nodeDef = Application().GetFactory().CreateICENodeDef(L"nest_GetPositionArray");

   CStatus st;

	st = nodeDef.PutThreadingModel(XSI::siICENodeSingleThreading);
	st.AssertSucceeded( ) ;

   // Add input ports and groups.
   st = nodeDef.AddPortGroup(Array_ID_G_100);
   st.AssertSucceeded( ) ;

   st = nodeDef.AddInputPort(Array_ID_IN_Vector,Array_ID_G_100,siICENodeDataVector3,siICENodeStructureSingle,siICENodeContextAny,L"Vector",L"Vector",CVector3f(0,0,0),Array_ID_UNDEF,Array_ID_UNDEF,Array_ID_CTXT_CNS);
   st.AssertSucceeded( ) ;

   // Add output ports and groups.
   st = nodeDef.AddPortGroup(Array_ID_G_300);
   st.AssertSucceeded( ) ;

   st = nodeDef.AddOutputPort(Array_ID_OUT_Result,Array_ID_G_300,siICENodeDataVector3,siICENodeStructureArray,siICENodeContextSingleton,L"Array",L"Array",Array_ID_UNDEF,Array_ID_UNDEF,Array_ID_UNDEF);
   st.AssertSucceeded( ) ;

   PluginItem nodeItem = in_reg.RegisterICENode(nodeDef);
   nodeItem.PutCategories(L"Array");

   return CStatus::OK;
}

CStatus Register_nest_LatticeNode( PluginRegistrar& in_reg )
{
   ICENodeDef nodeDef;
   nodeDef = Application().GetFactory().CreateICENodeDef(L"nest_LatticeDeform");

   CStatus st;

	st = nodeDef.PutThreadingModel(XSI::siICENodeMultiThreading);
	st.AssertSucceeded( ) ;

   // Add input ports and groups.
   st = nodeDef.AddPortGroup(Lattice_ID_G_100);
   st.AssertSucceeded( ) ;

   st = nodeDef.AddInputPort(Lattice_ID_IN_Point,Lattice_ID_G_100,siICENodeDataVector3,siICENodeStructureAny,siICENodeContextAny,L"Point",L"Point",CVector3f(0,0,0),Lattice_ID_UNDEF,Lattice_ID_STRUCT_CNS,Lattice_ID_CTXT_CNS);
   st.AssertSucceeded( ) ;
   st = nodeDef.AddInputPort(Lattice_ID_IN_Subdivision,Lattice_ID_G_100,siICENodeDataVector3,siICENodeStructureSingle,siICENodeContextSingleton,L"Subdivision",L"Subdivision",CVector3f(1,1,1),Lattice_ID_UNDEF,Lattice_ID_UNDEF,Lattice_ID_UNDEF);
   st.AssertSucceeded( ) ;
   st = nodeDef.AddInputPort(Lattice_ID_IN_Step,Lattice_ID_G_100,siICENodeDataVector3,siICENodeStructureSingle,siICENodeContextSingleton,L"Step",L"Step",CVector3f(1,1,1),Lattice_ID_UNDEF,Lattice_ID_UNDEF,Lattice_ID_UNDEF);
   st.AssertSucceeded( ) ;
   st = nodeDef.AddInputPort(Lattice_ID_IN_Reference,Lattice_ID_G_100,siICENodeDataVector3,siICENodeStructureArray,siICENodeContextSingleton,L"References",L"References",CVector3f(1,1,1),Lattice_ID_UNDEF,Lattice_ID_UNDEF,Lattice_ID_UNDEF);
   st.AssertSucceeded( ) ;
   st = nodeDef.AddInputPort(Lattice_ID_IN_Current,Lattice_ID_G_100,siICENodeDataVector3,siICENodeStructureArray,siICENodeContextSingleton,L"Currents",L"Currents",CVector3f(1,1,1),Lattice_ID_UNDEF,Lattice_ID_UNDEF,Lattice_ID_UNDEF);
   st.AssertSucceeded( ) ;

   // Add output ports and groups.
   st = nodeDef.AddPortGroup(Lattice_ID_G_300);
   st.AssertSucceeded( ) ;

   st = nodeDef.AddOutputPort(Lattice_ID_OUT_Result,Lattice_ID_G_300,siICENodeDataVector3,siICENodeStructureAny,siICENodeContextAny,L"Deform",L"Deform",Lattice_ID_UNDEF,Lattice_ID_STRUCT_CNS,Lattice_ID_CTXT_CNS);
   st.AssertSucceeded( ) ;

   PluginItem nodeItem = in_reg.RegisterICENode(nodeDef);
   nodeItem.PutCategories(L"Custom ICENode");

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus nest_GetPositionArray_Evaluate( ICENodeContext& in_ctxt )
{
   // The current output port being evaluated...
   ULONG out_portID = in_ctxt.GetEvaluatedOutputPortID( );

   switch( out_portID )
   {
      case Array_ID_OUT_Result:
      {
         // Get the output port array ...
         CDataArray2DVector3f outData( in_ctxt );

         // Get the input data buffers for each port
         CDataArrayVector3f InData( in_ctxt, Array_ID_IN_Vector );

         // We need a CIndexSet to iterate over the data
         LONG count = InData.GetCount();
         outData.Resize(0,InData.GetCount());

         for(LONG i=0;i<count;i++)
            outData[0][i] = InData[i];
      }
      break;

      // Other output ports...

   };

   return CStatus::OK;
}

long clampl(long a, long b, long c)
{
	if(a < b)
		return b;
	if(a > c)
		return c;
	return a;
}

float clampf(float a, float b, float c)
{
	if(a < b)
		return b;
	if(a > c)
		return c;
	return a;
}

long compose(long x, long y, long z, long yres, long zres)
{
	return x * yres * zres + y * zres + z;
}

XSIPLUGINCALLBACK CStatus nest_LatticeDeform_Evaluate( ICENodeContext& in_ctxt )
{
   // The current output port being evaluated...
   ULONG out_portID = in_ctxt.GetEvaluatedOutputPortID( );

   switch( out_portID )
   {
      case Array_ID_OUT_Result:
      {
         siICENodeDataType dataType;
         siICENodeStructureType struType;
         siICENodeContextType contType;
         in_ctxt.GetPortInfo(Lattice_ID_IN_Point,dataType,struType,contType);
			
         // get all of the data that is the same for any structure
         CDataArrayVector3f SubdivData( in_ctxt, Lattice_ID_IN_Subdivision );
         CDataArrayVector3f StepData( in_ctxt, Lattice_ID_IN_Step );
         CDataArray2DVector3f ReferenceData( in_ctxt, Lattice_ID_IN_Reference );
         CDataArray2DVector3f CurrentData( in_ctxt, Lattice_ID_IN_Current );
         CDataArray2DVector3f::Accessor ReferenceDataSub = ReferenceData[0];
         CDataArray2DVector3f::Accessor CurrentDataSub = CurrentData[0];

         // define the things we need to calculate
         long subdiv[3];
         subdiv[0] = long(floor(SubdivData[0].GetX()));
         subdiv[1] = long(floor(SubdivData[0].GetY()));
         subdiv[2] = long(floor(SubdivData[0].GetZ()));
         long subdiv1[3];
         subdiv1[0] = subdiv[0]+1;
         subdiv1[1] = subdiv[1]+1;
         subdiv1[2] = subdiv[2]+1;
         float step[3];
         step[0] = 1.0f / StepData[0].GetX();
         step[1] = 1.0f / StepData[0].GetY();
         step[2] = 1.0f / StepData[0].GetZ();
			float steplength = StepData[0].GetLength();
         long indexX[8];
         long indexY[8];
         long indexZ[8];
         long index[8];
         long lastIndex[3];
			lastIndex[0] = -1;
			lastIndex[1] = -1;
			lastIndex[2] = -1;
			CVector3f posCp;
			CVector3f pos;
         CVector3f diff[8];
         CVector3f motion[8];
         CVector3f motionScl[8];
			CVector3f deform;
         float weight[8];
			float xyz0[3];
			float xyz1[3];
			float weightSum;
         
         if(struType == siICENodeStructureSingle)
         {
            // two behaviours based on the datatype...
            // Get the output port array ...
            CDataArrayVector3f outData( in_ctxt );
   
            // Get the input data buffers for each port
            CDataArrayVector3f PointData( in_ctxt, Lattice_ID_IN_Point );

            // iterate each subset!
            CIndexSet IndexSet( in_ctxt );
            for(CIndexSet::Iterator it = IndexSet.Begin(); it.HasNext(); it.Next())
            {
               // first let's find the index inside the box!
					posCp.Set(PointData[it].GetX(),PointData[it].GetY(),PointData[it].GetZ());
					
					// substract the lowest corner
					pos.Sub(posCp,ReferenceDataSub[0]);
					pos.Set(pos.GetX() * step[0], pos.GetY() * step[1], pos.GetZ() * step[2]);
					xyz0[0] = pos.GetX() - floor(pos.GetX());
					xyz0[1] = pos.GetY() - floor(pos.GetY());
					xyz0[2] = pos.GetZ() - floor(pos.GetZ());

					xyz1[0] = 1.0 - xyz0[0];
					xyz1[1] = 1.0 - xyz0[1];
					xyz1[2] = 1.0 - xyz0[2];
					
					// calculate the indices (decomposed)
					indexX[0] = clampl(long(floor(pos.GetX())),0,subdiv[0]);
					indexY[0] = clampl(long(floor(pos.GetY())),0,subdiv[1]);
					indexZ[0] = clampl(long(floor(pos.GetZ())),0,subdiv[2]);
					
					if(lastIndex[0] != indexX[0] || lastIndex[1] != indexY[0] || lastIndex[2] != indexZ[0])
					{
						indexX[1] = clampl(indexX[0]+1	,0,subdiv[0]);
						indexY[1] = clampl(indexY[0]	,0,subdiv[1]);
						indexZ[1] = clampl(indexZ[0]	,0,subdiv[2]);
	
						indexX[2] = clampl(indexX[0]+1	,0,subdiv[0]);
						indexY[2] = clampl(indexY[0]+1	,0,subdiv[1]);
						indexZ[2] = clampl(indexZ[0]	,0,subdiv[2]);
	
						indexX[3] = clampl(indexX[0]+1	,0,subdiv[0]);
						indexY[3] = clampl(indexY[0]	,0,subdiv[1]);
						indexZ[3] = clampl(indexZ[0]+1	,0,subdiv[2]);
	
						indexX[4] = clampl(indexX[0]+1	,0,subdiv[0]);
						indexY[4] = clampl(indexY[0]+1	,0,subdiv[1]);
						indexZ[4] = clampl(indexZ[0]+1	,0,subdiv[2]);
						
						indexX[5] = clampl(indexX[0]	,0,subdiv[0]);
						indexY[5] = clampl(indexY[0]+1	,0,subdiv[1]);
						indexZ[5] = clampl(indexZ[0]	,0,subdiv[2]);
	
						indexX[6] = clampl(indexX[0]	,0,subdiv[0]);
						indexY[6] = clampl(indexY[0]	,0,subdiv[1]);
						indexZ[6] = clampl(indexZ[0]+1	,0,subdiv[2]);
	
						indexX[7] = clampl(indexX[0]	,0,subdiv[0]);
						indexY[7] = clampl(indexY[0]+1	,0,subdiv[1]);
						indexZ[7] = clampl(indexZ[0]+1	,0,subdiv[2]);
						
						for(int i=0;i<8;i++)
						{
							// compose the indices!
							index[i] = compose(indexX[i],indexY[i],indexZ[i],subdiv1[1],subdiv1[2]);
							
							// calculate the motions
							motion[i].Sub(CurrentDataSub[index[i]],ReferenceDataSub[index[i]]);
						}
					}
					else
					{
						// for performance, remember the last used index
						lastIndex[0] = indexX[0];
						lastIndex[1] = indexY[0];
						lastIndex[2] = indexZ[0];
					}

					// compute the weights
					weight[0] = xyz1[0] * xyz1[1] * xyz1[2];
					weight[1] = xyz0[0] * xyz1[1] * xyz1[2];
					weight[2] = xyz0[0] * xyz0[1] * xyz1[2];
					weight[3] = xyz0[0] * xyz1[1] * xyz0[2];
					weight[4] = xyz0[0] * xyz0[1] * xyz0[2];
					weight[5] = xyz1[0] * xyz0[1] * xyz1[2];
					weight[6] = xyz1[0] * xyz1[1] * xyz0[2];
					weight[7] = xyz1[0] * xyz0[1] * xyz0[2];

					// sum up all weighted motions
					deform.SetNull();
					for(int i=0;i<8;i++)
					{
						motionScl[i].Scale(weight[i],motion[i]);
						deform.AddInPlace(motionScl[i]);
					}
					
					// output the deformed position
					outData[it] = deform;
            }
         }
			else
         {
            // two behaviours based on the datatype...
            // Get the output port array ...
            CDataArray2DVector3f outData( in_ctxt );
   
            // Get the input data buffers for each port
            CDataArray2DVector3f PointData( in_ctxt, Lattice_ID_IN_Point );

            // iterate each subset!
            CIndexSet IndexSet( in_ctxt );
            for(CIndexSet::Iterator it = IndexSet.Begin(); it.HasNext(); it.Next())
            {
		         CDataArray2DVector3f::Accessor PointDataSub = PointData[it];
					long subCount = PointDataSub.GetCount();
					Application().LogMessage(CString((LONG)subCount));
					outData.Resize(it,subCount);
					for(long k=0;k<subCount;k++)
					{
						// first let's find the index inside the box!
						posCp.Set(PointDataSub[k].GetX(),PointDataSub[k].GetY(),PointDataSub[k].GetZ());
						
						// substract the lowest corner
						pos.Sub(posCp,ReferenceDataSub[0]);
						pos.Set(pos.GetX() * step[0], pos.GetY() * step[1], pos.GetZ() * step[2]);
						xyz0[0] = pos.GetX() - floor(pos.GetX());
						xyz0[1] = pos.GetY() - floor(pos.GetY());
						xyz0[2] = pos.GetZ() - floor(pos.GetZ());
	
						xyz1[0] = 1.0 - xyz0[0];
						xyz1[1] = 1.0 - xyz0[1];
						xyz1[2] = 1.0 - xyz0[2];
						
						// calculate the indices (decomposed)
						indexX[0] = clampl(long(floor(pos.GetX())),0,subdiv[0]);
						indexY[0] = clampl(long(floor(pos.GetY())),0,subdiv[1]);
						indexZ[0] = clampl(long(floor(pos.GetZ())),0,subdiv[2]);
						
						if(lastIndex[0] != indexX[0] || lastIndex[1] != indexY[0] || lastIndex[2] != indexZ[0])
						{
							indexX[1] = clampl(indexX[0]+1	,0,subdiv[0]);
							indexY[1] = clampl(indexY[0]	,0,subdiv[1]);
							indexZ[1] = clampl(indexZ[0]	,0,subdiv[2]);
		
							indexX[2] = clampl(indexX[0]+1	,0,subdiv[0]);
							indexY[2] = clampl(indexY[0]+1	,0,subdiv[1]);
							indexZ[2] = clampl(indexZ[0]	,0,subdiv[2]);
		
							indexX[3] = clampl(indexX[0]+1	,0,subdiv[0]);
							indexY[3] = clampl(indexY[0]	,0,subdiv[1]);
							indexZ[3] = clampl(indexZ[0]+1	,0,subdiv[2]);
		
							indexX[4] = clampl(indexX[0]+1	,0,subdiv[0]);
							indexY[4] = clampl(indexY[0]+1	,0,subdiv[1]);
							indexZ[4] = clampl(indexZ[0]+1	,0,subdiv[2]);
							
							indexX[5] = clampl(indexX[0]	,0,subdiv[0]);
							indexY[5] = clampl(indexY[0]+1	,0,subdiv[1]);
							indexZ[5] = clampl(indexZ[0]	,0,subdiv[2]);
		
							indexX[6] = clampl(indexX[0]	,0,subdiv[0]);
							indexY[6] = clampl(indexY[0]	,0,subdiv[1]);
							indexZ[6] = clampl(indexZ[0]+1	,0,subdiv[2]);
		
							indexX[7] = clampl(indexX[0]	,0,subdiv[0]);
							indexY[7] = clampl(indexY[0]+1	,0,subdiv[1]);
							indexZ[7] = clampl(indexZ[0]+1	,0,subdiv[2]);
							
							for(int i=0;i<8;i++)
							{
								// compose the indices!
								index[i] = compose(indexX[i],indexY[i],indexZ[i],subdiv1[1],subdiv1[2]);
								
								// calculate the motions
								motion[i].Sub(CurrentDataSub[index[i]],ReferenceDataSub[index[i]]);
							}
						}
						else
						{
							// for performance, remember the last used index
							lastIndex[0] = indexX[0];
							lastIndex[1] = indexY[0];
							lastIndex[2] = indexZ[0];
						}
	
						// compute the weights
						weight[0] = xyz1[0] * xyz1[1] * xyz1[2];
						weight[1] = xyz0[0] * xyz1[1] * xyz1[2];
						weight[2] = xyz0[0] * xyz0[1] * xyz1[2];
						weight[3] = xyz0[0] * xyz1[1] * xyz0[2];
						weight[4] = xyz0[0] * xyz0[1] * xyz0[2];
						weight[5] = xyz1[0] * xyz0[1] * xyz1[2];
						weight[6] = xyz1[0] * xyz1[1] * xyz0[2];
						weight[7] = xyz1[0] * xyz0[1] * xyz0[2];
	
						// sum up all weighted motions
						deform.SetNull();
						for(int i=0;i<8;i++)
						{
							motionScl[i].Scale(weight[i],motion[i]);
							deform.AddInPlace(motionScl[i]);
						}
						
						// output the deformed position
						outData[it][k] = deform;
					}
				}
         }
      }
      break;

      // Other output ports...

   };

   return CStatus::OK;
}
